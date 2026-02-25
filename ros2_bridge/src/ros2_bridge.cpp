// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — ROS2 Bridge: Implementation
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/ros2/ros2_bridge.h"

#include <cstring>
#include <sstream>

namespace thunderbird::ros2 {

using namespace thunderbird::ros2_helpers;

// ─── Construction ───────────────────────────────────────────────────────────

Ros2Bridge::Ros2Bridge(rclcpp::Node::SharedPtr node,
                       DeviceManager& device,
                       Ros2BridgeConfig config)
    : node_(std::move(node))
    , device_(device)
    , config_(std::move(config))
{
    // ── QoS profile: "sensor data" ──────────────────────────────────────
    //
    // BEST_EFFORT reliability avoids blocking the publisher when a
    // subscriber is slow or absent — critical for real-time sensor data.
    // VOLATILE durability means late-joining subscribers won't receive
    // stale data.  The small depth (default 5) keeps memory bounded.
    auto sensor_qos = rclcpp::SensorDataQoS();
    sensor_qos.keep_last(config_.qos_depth);

    // ── Create raw-sensor publishers ────────────────────────────────────
    if (config_.publish_raw_lidar) {
        lidar_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            config_.lidar_topic, sensor_qos);
    }
    if (config_.publish_raw_imu) {
        imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(
            config_.imu_topic, sensor_qos);
    }
    if (config_.publish_raw_camera) {
        camera_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
            config_.camera_topic, sensor_qos);
    }

    // ── Create synced-frame publishers ──────────────────────────────────
    if (config_.publish_synced) {
        sync_lidar_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            config_.sync_lidar_topic, sensor_qos);
        sync_camera_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
            config_.sync_camera_topic, sensor_qos);
        sync_imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(
            config_.sync_imu_topic, sensor_qos);
        // Info topic uses RELIABLE for diagnostics (small, infrequent).
        sync_info_pub_ = node_->create_publisher<std_msgs::msg::String>(
            config_.sync_info_topic, rclcpp::QoS(10));
    }

    RCLCPP_INFO(node_->get_logger(), "Ros2Bridge: publishers created");
}

Ros2Bridge::~Ros2Bridge() {
    stop();
}

// ─── Lifecycle ──────────────────────────────────────────────────────────────

void Ros2Bridge::start() {
    if (running_.exchange(true)) return;

    // ── Hook into the DataLayer Callback API ────────────────────────────
    //
    // The DataLayer fires these callbacks on the producer thread the
    // instant data arrives — lowest possible latency.  rclcpp::publish()
    // is thread-safe, so we can publish directly in the callback without
    // an extra queue or serialisation step.

    auto& dl = device_.data_layer();

    if (config_.publish_raw_lidar) {
        dl.onLidarFrame([this](const data::LidarFrame& f) { onLidar(f); });
    }
    if (config_.publish_raw_imu) {
        dl.onImuFrame([this](const data::ImuFrame& f) { onImu(f); });
    }
    if (config_.publish_raw_camera) {
        dl.onImageFrame([this](const data::ImageFrame& f) { onCamera(f); });
    }

    // ── Hook into the TimeSyncEngine Callback API ───────────────────────
    //
    // SyncedFrame bundles arrive at the LiDAR rate (~10 Hz).  We publish
    // the component messages on dedicated /synced_frame/* topics so that
    // downstream nodes can rely on all three being stamped to the same
    // reference epoch.

    if (config_.publish_synced) {
        device_.on_synced_frame(
            [this](const data::SyncedFrame& sf) { onSyncedFrame(sf); });
    }

    RCLCPP_INFO(node_->get_logger(), "Ros2Bridge: started (callback mode)");
}

void Ros2Bridge::stop() {
    if (!running_.exchange(false)) return;

    // Unregister callbacks by setting them to nullptr.
    auto& dl = device_.data_layer();
    dl.onLidarFrame(nullptr);
    dl.onImuFrame(nullptr);
    dl.onImageFrame(nullptr);

    // TimeSyncEngine callback can be cleared by registering nullptr.
    device_.time_sync().onSyncedFrame(nullptr);

    RCLCPP_INFO(node_->get_logger(), "Ros2Bridge: stopped");
}

// ─── Pull-mode publishing ───────────────────────────────────────────────────

size_t Ros2Bridge::publishPending() {
    size_t count = 0;
    auto& dl = device_.data_layer();

    // Drain LiDAR queue
    if (lidar_pub_) {
        while (auto frame = dl.getNextLidarFrame()) {
            auto msg = buildPointCloud2(*frame, config_.lidar_frame_id);
            lidar_pub_->publish(msg);
            lidar_count_.fetch_add(1, std::memory_order_relaxed);
            ++count;
        }
    }

    // Drain IMU queue
    if (imu_pub_) {
        while (auto frame = dl.getNextImuFrame()) {
            auto msg = buildImu(*frame, config_.imu_frame_id);
            imu_pub_->publish(msg);
            imu_count_.fetch_add(1, std::memory_order_relaxed);
            ++count;
        }
    }

    // Drain Camera queue
    if (camera_pub_) {
        while (auto frame = dl.getNextImageFrame()) {
            auto msg = buildImage(*frame, config_.camera_frame_id);
            camera_pub_->publish(msg);
            camera_count_.fetch_add(1, std::memory_order_relaxed);
            ++count;
        }
    }

    // Drain SyncedFrame queue
    if (sync_lidar_pub_) {
        while (auto sf = device_.time_sync().getNextSyncedFrame()) {
            onSyncedFrame(*sf);
            ++count;
        }
    }

    return count;
}

// ─── Statistics ─────────────────────────────────────────────────────────────

Ros2BridgeStats Ros2Bridge::stats() const noexcept {
    return {
        lidar_count_.load(std::memory_order_relaxed),
        imu_count_.load(std::memory_order_relaxed),
        camera_count_.load(std::memory_order_relaxed),
        synced_count_.load(std::memory_order_relaxed),
    };
}

// ─── Callback handlers ─────────────────────────────────────────────────────

void Ros2Bridge::onLidar(const data::LidarFrame& f) {
    if (!running_ || !lidar_pub_) return;
    auto msg = buildPointCloud2(f, config_.lidar_frame_id);
    lidar_pub_->publish(msg);
    lidar_count_.fetch_add(1, std::memory_order_relaxed);
}

void Ros2Bridge::onImu(const data::ImuFrame& f) {
    if (!running_ || !imu_pub_) return;
    auto msg = buildImu(f, config_.imu_frame_id);
    imu_pub_->publish(msg);
    imu_count_.fetch_add(1, std::memory_order_relaxed);
}

void Ros2Bridge::onCamera(const data::ImageFrame& f) {
    if (!running_ || !camera_pub_) return;
    auto msg = buildImage(f, config_.camera_frame_id);
    camera_pub_->publish(msg);
    camera_count_.fetch_add(1, std::memory_order_relaxed);
}

void Ros2Bridge::onSyncedFrame(const data::SyncedFrame& sf) {
    if (!running_) return;

    // ── Publish LiDAR component ─────────────────────────────────────────
    if (sync_lidar_pub_) {
        auto msg = buildPointCloud2(sf.lidar, config_.lidar_frame_id);
        sync_lidar_pub_->publish(msg);
    }

    // ── Publish Camera component (if matched) ───────────────────────────
    if (sync_camera_pub_ && sf.camera) {
        auto msg = buildImage(*sf.camera, config_.camera_frame_id);
        sync_camera_pub_->publish(msg);
    }

    // ── Publish representative IMU sample ───────────────────────────────
    //
    // We publish the *first* IMU sample in the block as the single Imu
    // message.  Downstream nodes that need the full block can subscribe
    // to the raw /imu/data topic and correlate by timestamp, or a future
    // custom message can carry the full vector.
    if (sync_imu_pub_ && !sf.imu_block.empty()) {
        auto msg = buildImu(sf.imu_block.front(), config_.imu_frame_id);
        sync_imu_pub_->publish(msg);
    }

    // ── Publish sync diagnostics ────────────────────────────────────────
    if (sync_info_pub_) {
        std_msgs::msg::String info;
        std::ostringstream os;
        os << "seq=" << sf.sequence
           << " quality=" << sf.sync_quality
           << " offset_ns=" << sf.lidar_camera_offset_ns
           << " imu_block_size=" << sf.imu_block.size()
           << " camera=" << (sf.camera ? "yes" : "no");
        info.data = os.str();
        sync_info_pub_->publish(info);
    }

    synced_count_.fetch_add(1, std::memory_order_relaxed);
}

// ─── Message builders ───────────────────────────────────────────────────────

sensor_msgs::msg::PointCloud2
Ros2Bridge::buildPointCloud2(const data::LidarFrame& frame,
                             const std::string& frame_id)
{
    sensor_msgs::msg::PointCloud2 msg;

    // Header
    auto stamp = to_stamp(frame.timestamp_ns);
    msg.header.stamp.sec     = stamp.sec;
    msg.header.stamp.nanosec = stamp.nanosec;
    msg.header.frame_id      = frame_id;

    // Dimensions
    msg.height       = 1;
    msg.width        = static_cast<uint32_t>(frame.points.size());
    msg.is_dense     = true;
    msg.is_bigendian = false;

    // ── Field descriptors ───────────────────────────────────────────────
    //
    // We include x, y, z, intensity, and timestamp_offset_ns so that
    // downstream nodes can perform per-point motion compensation using
    // the hardware-level timing information.
    auto fields = lidar_point_fields();
    msg.fields.resize(fields.size());
    for (size_t i = 0; i < fields.size(); ++i) {
        msg.fields[i].name     = fields[i].name;
        msg.fields[i].offset   = fields[i].offset;
        msg.fields[i].datatype = static_cast<uint8_t>(fields[i].datatype);
        msg.fields[i].count    = fields[i].count;
    }

    msg.point_step = kPointStep;
    msg.row_step   = kPointStep * msg.width;

    // Serialise point data into the flat byte buffer.
    msg.data = serialise_points(frame.points);

    return msg;
}

sensor_msgs::msg::Imu
Ros2Bridge::buildImu(const data::ImuFrame& frame,
                     const std::string& frame_id)
{
    sensor_msgs::msg::Imu msg;

    auto stamp = to_stamp(frame.timestamp_ns);
    msg.header.stamp.sec     = stamp.sec;
    msg.header.stamp.nanosec = stamp.nanosec;
    msg.header.frame_id      = frame_id;

    msg.linear_acceleration.x = frame.accel[0];
    msg.linear_acceleration.y = frame.accel[1];
    msg.linear_acceleration.z = frame.accel[2];

    msg.angular_velocity.x = frame.gyro[0];
    msg.angular_velocity.y = frame.gyro[1];
    msg.angular_velocity.z = frame.gyro[2];

    // Orientation is not provided by the IMU — mark as unknown.
    // ROS convention: set covariance[0] = -1 to indicate "don't use".
    msg.orientation_covariance[0] = -1.0;

    return msg;
}

sensor_msgs::msg::Image
Ros2Bridge::buildImage(const data::ImageFrame& frame,
                       const std::string& frame_id)
{
    sensor_msgs::msg::Image msg;

    auto stamp = to_stamp(frame.timestamp_ns);
    msg.header.stamp.sec     = stamp.sec;
    msg.header.stamp.nanosec = stamp.nanosec;
    msg.header.frame_id      = frame_id;

    msg.width    = frame.width;
    msg.height   = frame.height;
    msg.encoding = pixel_format_to_encoding(frame.format);
    msg.step     = frame.stride;

    // ── Zero-copy pixel access ──────────────────────────────────────────
    //
    // The SDK's ImageFrame stores pixels in a shared_ptr<vector<uint8_t>>.
    // We must copy into the ROS2 message's data field because sensor_msgs
    // owns its buffer.  In a production system with intra-process comms
    // this could be replaced with a loaned message to avoid the copy.
    if (frame.data) {
        msg.data = *frame.data;
    }

    msg.is_bigendian = 0;

    return msg;
}

} // namespace thunderbird::ros2
