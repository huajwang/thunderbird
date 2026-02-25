// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — ROS2 Bridge: Header
// ─────────────────────────────────────────────────────────────────────────────
//
//   ┌──────────────────────┐          ┌──────────────────────────────────┐
//   │   DeviceManager      │          │         ROS2 Graph               │
//   │  ┌──────────────┐    │          │                                  │
//   │  │  DataLayer    │───callback──►│  /thunderbird/lidar/points       │
//   │  │  (Pull+CB)   │    │          │  /thunderbird/imu/data           │
//   │  └──────────────┘    │          │  /thunderbird/camera/image       │
//   │  ┌──────────────┐    │          │  /thunderbird/synced_frame       │
//   │  │TimeSyncEngine│───callback──►│   (PointCloud2+Image+Imu block)  │
//   │  └──────────────┘    │          │                                  │
//   └──────────────────────┘          └──────────────────────────────────┘
//
// Design rationale:
//
//   • **Ros2Bridge** is a composable helper, *not* a Node subclass.
//     It accepts any rclcpp::Node::SharedPtr so the user can embed it
//     in their own node or use the convenience standalone node we ship.
//
//   • **Callback-driven publishing**: the bridge registers lightweight
//     callbacks on the DataLayer and TimeSyncEngine that convert SDK
//     types → ROS2 messages and publish.  This means data flows at the
//     SDK's native cadence with no extra polling thread.
//
//   • **Optional Pull-API mode**: If the user prefers, they can call
//     `publishPending()` from a timer or render loop to drain the SDK
//     queues and publish in batch.  This is useful when the user wants
//     explicit control over the publish rate.
//
//   • **QoS**: we use the ROS2 "sensor data" profile (BEST_EFFORT
//     reliability, VOLATILE durability, small queue depth).  This matches
//     real-time sensor data patterns and avoids back-pressure from slow
//     subscribers.  Custom QoS can be provided in Ros2BridgeConfig.
//
//   • **SyncedFrame**: published as three coordinated messages on a
//     stamped `/thunderbird/synced_frame/` group:
//       - `/thunderbird/synced_frame/lidar`   PointCloud2
//       - `/thunderbird/synced_frame/camera`  Image
//       - `/thunderbird/synced_frame/imu`     Imu  (first sample in block)
//     Plus a diagnostic `/thunderbird/synced_frame/info` String with
//     quality/offset metadata.  A production system would define a custom
//     .msg — this approach works without message generation.
//
//   • **Thread safety**: rclcpp publishers are thread-safe for publish().
//     The callbacks may fire on any producer thread; publish() is safe
//     to call from there.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

// ROS2 headers
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

// SDK headers
#include "thunderbird/device_manager.h"
#include "thunderbird/ros2_helpers.h"

#include <atomic>
#include <memory>
#include <string>

namespace thunderbird::ros2 {

// ─── Configuration ──────────────────────────────────────────────────────────

struct Ros2BridgeConfig {
    // ── Topic names ─────────────────────────────────────────────────────
    std::string lidar_topic   = "/thunderbird/lidar/points";
    std::string imu_topic     = "/thunderbird/imu/data";
    std::string camera_topic  = "/thunderbird/camera/image";

    // Synced-frame sub-topics
    std::string sync_lidar_topic  = "/thunderbird/synced_frame/lidar";
    std::string sync_camera_topic = "/thunderbird/synced_frame/camera";
    std::string sync_imu_topic    = "/thunderbird/synced_frame/imu";
    std::string sync_info_topic   = "/thunderbird/synced_frame/info";

    // ── Frame IDs ───────────────────────────────────────────────────────
    std::string lidar_frame_id  = "thunderbird_lidar";
    std::string imu_frame_id    = "thunderbird_imu";
    std::string camera_frame_id = "thunderbird_camera";

    // ── Publish enables ─────────────────────────────────────────────────
    bool publish_raw_lidar  = true;
    bool publish_raw_imu    = true;
    bool publish_raw_camera = true;
    bool publish_synced     = true;

    // ── QoS ─────────────────────────────────────────────────────────────
    /// Queue depth for all sensor publishers.  Uses BEST_EFFORT +
    /// VOLATILE (sensor_data profile) by default.
    size_t qos_depth = ros2_helpers::kSensorQosDepth;
};

// ─── Statistics ─────────────────────────────────────────────────────────────

struct Ros2BridgeStats {
    uint64_t lidar_published{0};
    uint64_t imu_published{0};
    uint64_t camera_published{0};
    uint64_t synced_published{0};
};

// ─────────────────────────────────────────────────────────────────────────────

/// Publishes Thunderbird SDK sensor data to standard ROS2 topics.
///
/// Typical usage:
/// @code
///   auto node = std::make_shared<rclcpp::Node>("my_node");
///   thunderbird::DeviceManager dev;
///   dev.connect(); dev.start();
///
///   thunderbird::ros2::Ros2Bridge bridge(node, dev);
///   bridge.start();       // registers callbacks, publishing begins
///   rclcpp::spin(node);   // process ROS2
///   bridge.stop();
/// @endcode
class Ros2Bridge {
public:
    /// Construct the bridge.
    ///
    /// @param node    A live rclcpp Node used to create publishers.
    /// @param device  A connected & streaming DeviceManager.
    /// @param config  Optional topic / QoS / enable configuration.
    Ros2Bridge(rclcpp::Node::SharedPtr node,
               DeviceManager& device,
               Ros2BridgeConfig config = {});

    ~Ros2Bridge();

    // non-copyable, non-movable (captures `this` in callbacks)
    Ros2Bridge(const Ros2Bridge&) = delete;
    Ros2Bridge& operator=(const Ros2Bridge&) = delete;

    // ── Lifecycle ───────────────────────────────────────────────────────

    /// Register DataLayer + TimeSyncEngine callbacks and begin publishing.
    void start();

    /// Unregister callbacks and stop publishing.
    void stop();

    [[nodiscard]] bool running() const noexcept { return running_.load(); }

    // ── Pull-mode publishing ────────────────────────────────────────────
    //
    // Instead of callback-driven publishing, the user can call this from
    // a timer to drain the SDK queues and publish all pending frames.
    // Useful when you want to control the publish cadence explicitly.

    /// Drain SDK pull queues and publish all pending frames.
    /// Returns the total number of messages published in this call.
    size_t publishPending();

    // ── Statistics ──────────────────────────────────────────────────────

    [[nodiscard]] Ros2BridgeStats stats() const noexcept;

private:
    // ── Message builders ────────────────────────────────────────────────
    //
    // Each builder converts an SDK public type to a ROS2 message.
    // They are static so they can be unit-tested independently.

    [[nodiscard]] static sensor_msgs::msg::PointCloud2
    buildPointCloud2(const data::LidarFrame& frame,
                     const std::string& frame_id);

    [[nodiscard]] static sensor_msgs::msg::Imu
    buildImu(const data::ImuFrame& frame,
             const std::string& frame_id);

    [[nodiscard]] static sensor_msgs::msg::Image
    buildImage(const data::ImageFrame& frame,
               const std::string& frame_id);

    // ── Callback handlers ───────────────────────────────────────────────

    void onLidar(const data::LidarFrame& f);
    void onImu(const data::ImuFrame& f);
    void onCamera(const data::ImageFrame& f);
    void onSyncedFrame(const data::SyncedFrame& sf);

    // ── State ───────────────────────────────────────────────────────────

    rclcpp::Node::SharedPtr node_;
    DeviceManager&          device_;
    Ros2BridgeConfig        config_;
    std::atomic<bool>       running_{false};

    // Raw-sensor publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr         imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr       camera_pub_;

    // Synced-frame publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sync_lidar_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr       sync_camera_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr         sync_imu_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr         sync_info_pub_;

    // Counters
    std::atomic<uint64_t> lidar_count_{0};
    std::atomic<uint64_t> imu_count_{0};
    std::atomic<uint64_t> camera_count_{0};
    std::atomic<uint64_t> synced_count_{0};
};

} // namespace thunderbird::ros2
