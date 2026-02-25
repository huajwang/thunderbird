// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — ROS2 Bridge Node (Step 4)
// ─────────────────────────────────────────────────────────────────────────────
//
// A self-contained ROS2 executable that:
//   1. Creates an rclcpp Node.
//   2. Instantiates the Thunderbird DeviceManager.
//   3. Wires the Ros2Bridge to publish raw + synced data.
//   4. Spins until shutdown (Ctrl-C / lifecycle).
//
// Published topics:
//   /thunderbird/lidar/points         sensor_msgs/PointCloud2
//   /thunderbird/imu/data             sensor_msgs/Imu
//   /thunderbird/camera/image         sensor_msgs/Image
//   /thunderbird/synced_frame/lidar   sensor_msgs/PointCloud2
//   /thunderbird/synced_frame/camera  sensor_msgs/Image
//   /thunderbird/synced_frame/imu     sensor_msgs/Imu
//   /thunderbird/synced_frame/info    std_msgs/String
//
// Parameters (all optional, declare_parameter defaults used):
//   lidar_hz    (double, default 10.0)
//   imu_hz      (double, default 200.0)
//   camera_fps  (double, default 30.0)
//
// Build:
//   colcon build --packages-select thunderbird_ros2_bridge
//
// Run:
//   ros2 run thunderbird_ros2_bridge thunderbird_ros2_node
//   ros2 run thunderbird_ros2_bridge thunderbird_ros2_node \
//       --ros-args -p lidar_hz:=20.0
//
// ─────────────────────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include "thunderbird/ros2/ros2_bridge.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // ── Create the ROS2 node ────────────────────────────────────────────
    auto node = std::make_shared<rclcpp::Node>("thunderbird_node");

    // ── Read parameters for sensor configuration ────────────────────────
    thunderbird::DeviceConfig dev_cfg;
    dev_cfg.lidar_hz   = node->declare_parameter("lidar_hz",   10.0);
    dev_cfg.imu_hz     = node->declare_parameter("imu_hz",     200.0);
    dev_cfg.camera_fps = node->declare_parameter("camera_fps", 30.0);

    // ── Create the SDK DeviceManager ────────────────────────────────────
    thunderbird::DeviceManager device(dev_cfg);

    auto status = device.connect();
    if (status != thunderbird::Status::OK) {
        RCLCPP_FATAL(node->get_logger(), "DeviceManager::connect() failed: %s",
                     thunderbird::status_string(status));
        return 1;
    }

    status = device.start();
    if (status != thunderbird::Status::OK) {
        RCLCPP_FATAL(node->get_logger(), "DeviceManager::start() failed: %s",
                     thunderbird::status_string(status));
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "SDK connected and streaming.");

    // ── Create the ROS2 bridge ──────────────────────────────────────────
    //
    // The bridge registers callbacks on the DataLayer and TimeSyncEngine.
    // Data flows:  sensor driver → DataLayer callback → Ros2Bridge → publish.
    thunderbird::ros2::Ros2BridgeConfig bridge_cfg;
    // All defaults are fine — raw + synced publishing enabled.

    thunderbird::ros2::Ros2Bridge bridge(node, device, bridge_cfg);
    bridge.start();

    RCLCPP_INFO(node->get_logger(),
                "Ros2Bridge started — publishing to ROS2 topics.");

    // ── Spin until shutdown ─────────────────────────────────────────────
    rclcpp::spin(node);

    // ── Clean shutdown ──────────────────────────────────────────────────
    bridge.stop();
    device.stop();
    device.disconnect();

    auto st = bridge.stats();
    RCLCPP_INFO(node->get_logger(),
        "Shutdown. Published: lidar=%lu imu=%lu camera=%lu synced=%lu",
        static_cast<unsigned long>(st.lidar_published),
        static_cast<unsigned long>(st.imu_published),
        static_cast<unsigned long>(st.camera_published),
        static_cast<unsigned long>(st.synced_published));

    rclcpp::shutdown();
    return 0;
}
