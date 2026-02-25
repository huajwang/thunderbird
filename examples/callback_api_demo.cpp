// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Callback API demo
// ─────────────────────────────────────────────────────────────────────────────
//
// Demonstrates the Data Abstraction Layer *Callback API*: the application
// registers handlers that fire on the producer thread the instant new data
// arrives.  This is the lowest-latency consumption pattern.
//
// Build:  cmake --build build --target callback_api_demo
// Run:    ./build/examples/callback_api_demo
//
// ─────────────────────────────────────────────────────────────────────────────

#include <thunderbird/device_manager.h>
#include <thunderbird/data_layer.h>

#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <cstdio>

using namespace thunderbird::data;

int main() {
    std::cout << "=== Callback API Demo ===" << std::endl;

    // Atomic counters — updated from producer threads, read from main.
    std::atomic<int> lidar_n{0};
    std::atomic<int> imu_n{0};
    std::atomic<int> cam_n{0};

    // ── 1. Create and connect ───────────────────────────────────────────
    thunderbird::DeviceConfig cfg;
    cfg.lidar_hz   = 10.0;
    cfg.imu_hz     = 100.0;
    cfg.camera_fps = 5.0;

    thunderbird::DeviceManager dev(cfg);
    thunderbird::Status s = dev.connect();
    if (s != thunderbird::Status::OK) {
        std::cerr << "connect() failed: " << thunderbird::status_string(s) << "\n";
        return 1;
    }
    std::cout << "Connected to: " << dev.device_info().model_name << "\n\n";

    // ── 2. Register callbacks (before starting!) ────────────────────────
    auto& layer = dev.data_layer();

    layer.onLidarFrame([&](const LidarFrame& f) {
        int n = ++lidar_n;
        if (n <= 3) {
            std::printf("[CB] LiDAR  seq=%u  pts=%zu  ts=%lld ns\n",
                        f.sequence,
                        f.points.size(),
                        static_cast<long long>(f.timestamp_ns));
            if (!f.points.empty()) {
                const auto& p = f.points.front();
                std::printf("       first point: (%.2f, %.2f, %.2f) "
                            "I=%.0f  offset=%d ns\n",
                            p.x, p.y, p.z, p.intensity,
                            p.timestamp_offset_ns);
            }
        }
    });

    layer.onImuFrame([&](const ImuFrame& f) {
        int n = ++imu_n;
        if (n <= 5) {
            std::printf("[CB] IMU    accel=(%.3f, %.3f, %.3f)  "
                        "gyro=(%.4f, %.4f, %.4f)  ts=%lld ns\n",
                        f.accel[0], f.accel[1], f.accel[2],
                        f.gyro[0],  f.gyro[1],  f.gyro[2],
                        static_cast<long long>(f.timestamp_ns));
        }
    });

    layer.onImageFrame([&](const ImageFrame& f) {
        int n = ++cam_n;
        if (n <= 3) {
            std::printf("[CB] Camera seq=%u  %ux%u  %zu bytes  ts=%lld ns\n",
                        f.sequence, f.width, f.height,
                        f.pixels().size(),
                        static_cast<long long>(f.timestamp_ns));
        }
    });

    // ── 3. Start streaming ──────────────────────────────────────────────
    s = dev.start();
    if (s != thunderbird::Status::OK) {
        std::cerr << "start() failed: " << thunderbird::status_string(s) << "\n";
        return 1;
    }
    std::cout << "Streaming for 1 second ...\n\n";

    // ── 4. Let callbacks fire for 1 second ──────────────────────────────
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // ── 5. Stop and report ──────────────────────────────────────────────
    dev.stop();
    dev.disconnect();

    std::cout << "\n── Summary ────────────────────────────────────────────\n";
    std::printf("  LiDAR  frames received : %d\n", lidar_n.load());
    std::printf("  IMU    frames received : %d\n", imu_n.load());
    std::printf("  Camera frames received : %d\n", cam_n.load());
    std::printf("  LiDAR  dropped         : %llu\n",
                static_cast<unsigned long long>(layer.lidarDropped()));
    std::printf("  IMU    dropped         : %llu\n",
                static_cast<unsigned long long>(layer.imuDropped()));
    std::printf("  Camera dropped         : %llu\n",
                static_cast<unsigned long long>(layer.cameraDropped()));

    std::cout << "\nDone.\n";
    return 0;
}
