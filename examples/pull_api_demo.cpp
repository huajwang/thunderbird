// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Pull API demo
// ─────────────────────────────────────────────────────────────────────────────
//
// Demonstrates the Data Abstraction Layer *Pull API*: the application polls
// for sensor frames at its own cadence using getNext*Frame().
//
// Build:  cmake --build build --target pull_api_demo
// Run:    ./build/examples/pull_api_demo
//
// ─────────────────────────────────────────────────────────────────────────────

#include <thunderbird/device_manager.h>
#include <thunderbird/data_layer.h>

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <cstdio>

using namespace thunderbird::data;

int main() {
    std::cout << "=== Pull API Demo ===" << std::endl;

    // ── 1. Create and connect ───────────────────────────────────────────
    thunderbird::DeviceConfig cfg;
    cfg.lidar_hz    = 10.0;
    cfg.imu_hz      = 100.0;   // lower rate for demo readability
    cfg.camera_fps  = 5.0;

    thunderbird::DeviceManager dev(cfg);
    thunderbird::Status s = dev.connect();
    if (s != thunderbird::Status::OK) {
        std::cerr << "connect() failed: " << thunderbird::status_string(s) << "\n";
        return 1;
    }
    std::cout << "Connected to: " << dev.device_info().model_name << "\n\n";

    // ── 2. Start streaming ──────────────────────────────────────────────
    s = dev.start();
    if (s != thunderbird::Status::OK) {
        std::cerr << "start() failed: " << thunderbird::status_string(s) << "\n";
        return 1;
    }

    // ── 3. Let some data accumulate ─────────────────────────────────────
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // ── 4. Drain the queues using the Pull API ──────────────────────────
    auto& layer = dev.data_layer();

    std::cout << "── LiDAR frames ───────────────────────────────────────\n";
    int lidar_count = 0;
    while (auto frame = layer.getNextLidarFrame()) {
        ++lidar_count;
        std::printf("  LiDAR seq=%u  ts=%lld ns  points=%zu",
                    frame->sequence,
                    static_cast<long long>(frame->timestamp_ns),
                    frame->points.size());

        // Show first point's per-point timestamp offset.
        if (!frame->points.empty()) {
            auto span = frame->point_span();
            const auto& p0 = span.front();
            std::printf("  p0=(%.2f, %.2f, %.2f) offset=%d ns",
                        p0.x, p0.y, p0.z, p0.timestamp_offset_ns);
        }
        std::printf("\n");

        if (lidar_count >= 5) {
            std::printf("  ... (showing first 5)\n");
            break;
        }
    }

    std::cout << "\n── IMU frames ─────────────────────────────────────────\n";
    int imu_count = 0;
    while (auto frame = layer.getNextImuFrame()) {
        ++imu_count;
        std::printf("  IMU ts=%lld ns  accel=(%.3f, %.3f, %.3f)  gyro=(%.4f, %.4f, %.4f)\n",
                    static_cast<long long>(frame->timestamp_ns),
                    frame->accel[0], frame->accel[1], frame->accel[2],
                    frame->gyro[0],  frame->gyro[1],  frame->gyro[2]);

        if (imu_count >= 10) {
            std::printf("  ... (showing first 10)\n");
            break;
        }
    }

    std::cout << "\n── Camera frames ──────────────────────────────────────\n";
    int cam_count = 0;
    while (auto frame = layer.getNextImageFrame()) {
        ++cam_count;
        std::printf("  Camera seq=%u  ts=%lld ns  %ux%u  stride=%u  pixels=%zu bytes\n",
                    frame->sequence,
                    static_cast<long long>(frame->timestamp_ns),
                    frame->width, frame->height,
                    frame->stride,
                    frame->pixels().size());

        if (cam_count >= 5) {
            std::printf("  ... (showing first 5)\n");
            break;
        }
    }

    // ── 5. Show drop statistics ─────────────────────────────────────────
    std::cout << "\n── Queue statistics ────────────────────────────────────\n";
    std::printf("  LiDAR  : %zu queued, %llu dropped\n",
                layer.lidarQueued(), static_cast<unsigned long long>(layer.lidarDropped()));
    std::printf("  IMU    : %zu queued, %llu dropped\n",
                layer.imuQueued(), static_cast<unsigned long long>(layer.imuDropped()));
    std::printf("  Camera : %zu queued, %llu dropped\n",
                layer.cameraQueued(), static_cast<unsigned long long>(layer.cameraDropped()));

    // ── 6. Demonstrate blocking pull with timeout ───────────────────────
    std::cout << "\n── Blocking pull (200 ms timeout) ─────────────────────\n";
    if (auto imu = layer.waitForImuFrame(std::chrono::milliseconds(200))) {
        std::printf("  Got IMU: accel=(%.3f, %.3f, %.3f)\n",
                    imu->accel[0], imu->accel[1], imu->accel[2]);
    } else {
        std::cout << "  (timed out — no IMU frame within 200 ms)\n";
    }

    // ── 7. Clean up ─────────────────────────────────────────────────────
    dev.stop();
    dev.disconnect();
    std::cout << "\nDone.\n";
    return 0;
}
