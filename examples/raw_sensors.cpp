// ─────────────────────────────────────────────────────────────────────────────
// Example 3 — Raw sensor deep-dive: inspect individual sensor data
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/thunderbird.h"

#include <chrono>
#include <cstdio>
#include <mutex>
#include <thread>

int main() {
    using namespace thunderbird;

    std::mutex print_mu;

    DeviceConfig cfg;
    cfg.lidar_hz   = 5.0;   // slower for readability
    cfg.imu_hz     = 50.0;
    cfg.camera_fps = 10.0;

    DeviceManager device(cfg);

    // ── LiDAR callback ─────────────────────────────────────────────────────
    device.on_lidar([&](std::shared_ptr<const LidarFrame> f) {
        std::lock_guard<std::mutex> lk(print_mu);
        std::printf("[LiDAR] seq=%u  pts=%zu  ts=%.6f s\n",
                    f->sequence_number, f->points.size(), f->timestamp.to_seconds());
        // Print first 3 points
        for (size_t i = 0; i < std::min<size_t>(3, f->points.size()); ++i) {
            auto& p = f->points[i];
            std::printf("        pt[%zu]  x=%.2f y=%.2f z=%.2f  i=%.0f ring=%u\n",
                        i, p.x, p.y, p.z, p.intensity, p.ring);
        }
    });

    // ── IMU callback ────────────────────────────────────────────────────────
    int imu_skip = 0;
    device.on_imu([&](std::shared_ptr<const ImuSample> s) {
        // Print every 50th sample to avoid flooding
        if (++imu_skip % 50 != 0) return;
        std::lock_guard<std::mutex> lk(print_mu);
        std::printf("[IMU]   accel=(%.3f, %.3f, %.3f) m/s²  gyro=(%.4f, %.4f, %.4f) rad/s  T=%.1f°C\n",
                    s->accel[0], s->accel[1], s->accel[2],
                    s->gyro[0],  s->gyro[1],  s->gyro[2],
                    s->temperature);
    });

    // ── Camera callback ─────────────────────────────────────────────────────
    device.on_camera([&](std::shared_ptr<const CameraFrame> f) {
        std::lock_guard<std::mutex> lk(print_mu);
        std::printf("[CAM]   seq=%u  %ux%u  %zu bytes  ts=%.6f s\n",
                    f->sequence_number, f->width, f->height,
                    f->data.size(), f->timestamp.to_seconds());
    });

    auto s = device.connect();
    if (s != Status::OK) { std::fprintf(stderr, "connect: %s\n", status_string(s)); return 1; }

    s = device.start();
    if (s != Status::OK) { std::fprintf(stderr, "start: %s\n", status_string(s)); return 1; }

    std::puts("=== Raw sensor output (2 seconds) ===\n");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    device.stop();
    device.disconnect();
    std::puts("\nDone.");
    return 0;
}
