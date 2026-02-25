// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — ROS2 Bridge Example: Pull-Mode with Timer
// ─────────────────────────────────────────────────────────────────────────────
//
// Demonstrates an alternative usage pattern where instead of callback-driven
// publishing, the user controls the publish cadence with a ROS2 timer that
// calls Ros2Bridge::publishPending() to drain the SDK queues.
//
// This pattern is useful when:
//   • You want to cap the ROS2 publish rate independently of sensor rate.
//   • You're integrating with a perception pipeline that runs at a fixed rate.
//   • You need to batch multiple messages in a single spin cycle.
//
// Build:  colcon build --packages-select thunderbird_ros2_bridge
// Run:    ros2 run thunderbird_ros2_bridge ros2_bridge_pull_demo
//
// ─────────────────────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include "thunderbird/ros2/ros2_bridge.h"

class PullBridgeNode : public rclcpp::Node {
public:
    PullBridgeNode() : Node("thunderbird_pull_bridge") {
        // ── SDK setup ───────────────────────────────────────────────────
        thunderbird::DeviceConfig dev_cfg;
        dev_cfg.lidar_hz   = declare_parameter("lidar_hz",   10.0);
        dev_cfg.imu_hz     = declare_parameter("imu_hz",     200.0);
        dev_cfg.camera_fps = declare_parameter("camera_fps", 30.0);

        device_ = std::make_unique<thunderbird::DeviceManager>(dev_cfg);
        device_->connect();
        device_->start();

        // ── Bridge setup (pull mode — no start() needed) ────────────────
        //
        // We deliberately do NOT call bridge.start() here, so no callbacks
        // are registered.  Instead, we drain the SDK queues manually.
        thunderbird::ros2::Ros2BridgeConfig bridge_cfg;
        bridge_cfg.publish_synced = false;  // synced via pull not supported yet

        bridge_ = std::make_unique<thunderbird::ros2::Ros2Bridge>(
            shared_from_this(), *device_, bridge_cfg);

        // ── Timer: drain and publish at 50 Hz ───────────────────────────
        timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            [this]() {
                auto n = bridge_->publishPending();
                if (n > 0) {
                    RCLCPP_DEBUG(get_logger(), "Published %zu messages", n);
                }
            });

        RCLCPP_INFO(get_logger(), "Pull-mode bridge running at 50 Hz.");
    }

    ~PullBridgeNode() override {
        if (bridge_) bridge_->stop();
        if (device_) { device_->stop(); device_->disconnect(); }
    }

private:
    std::unique_ptr<thunderbird::DeviceManager>     device_;
    std::unique_ptr<thunderbird::ros2::Ros2Bridge>  bridge_;
    rclcpp::TimerBase::SharedPtr                    timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PullBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
