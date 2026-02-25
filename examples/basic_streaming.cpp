// ─────────────────────────────────────────────────────────────────────────────
// Example 1 — Basic streaming: connect, stream all sensors, print stats
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/thunderbird.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>

int main() {
    using namespace thunderbird;

    std::atomic<uint64_t> lidar_count{0};
    std::atomic<uint64_t> imu_count{0};
    std::atomic<uint64_t> camera_count{0};

    DeviceConfig cfg;
    cfg.lidar_hz   = 10.0;
    cfg.imu_hz     = 200.0;
    cfg.camera_fps = 30.0;

    DeviceManager device(cfg);

    // Register raw callbacks
    device.on_lidar([&](auto /*frame*/) { ++lidar_count; });
    device.on_imu([&](auto /*sample*/) { ++imu_count; });
    device.on_camera([&](auto /*frame*/) { ++camera_count; });

    // Connect & start
    Status s = device.connect();
    if (s != Status::OK) {
        std::fprintf(stderr, "connect() failed: %s\n", status_string(s));
        return 1;
    }

    auto info = device.device_info();
    std::printf("Connected to %s  (S/N: %s, FW: %s)\n",
                info.model_name.c_str(),
                info.serial_number.c_str(),
                info.firmware_version.c_str());

    s = device.start();
    if (s != Status::OK) {
        std::fprintf(stderr, "start() failed: %s\n", status_string(s));
        return 1;
    }

    std::puts("Streaming for 5 seconds ...");

    for (int i = 0; i < 10; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::printf("  [%4.1fs]  LiDAR: %llu   IMU: %llu   Camera: %llu\n",
                    (i + 1) * 0.5,
                    static_cast<unsigned long long>(lidar_count.load()),
                    static_cast<unsigned long long>(imu_count.load()),
                    static_cast<unsigned long long>(camera_count.load()));
    }

    device.stop();
    device.disconnect();
    std::puts("Done.");
    return 0;
}
