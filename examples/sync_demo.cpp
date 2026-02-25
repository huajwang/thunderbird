// ─────────────────────────────────────────────────────────────────────────────
// Example 2 — Synchronized data: receive time-aligned bundles
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/thunderbird.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>

int main() {
    using namespace thunderbird;

    std::atomic<uint64_t> bundle_count{0};

    DeviceConfig cfg;
    cfg.lidar_hz   = 10.0;
    cfg.imu_hz     = 200.0;
    cfg.camera_fps = 30.0;
    cfg.sync.tolerance_ns = 60'000'000; // 60 ms

    DeviceManager device(cfg);

    // Register synchronized callback
    device.on_sync([&](std::shared_ptr<const SyncBundle> bundle) {
        ++bundle_count;

        // Print info about the bundle contents
        std::printf("Bundle #%llu  ref=%.6f s",
                    static_cast<unsigned long long>(bundle_count.load()),
                    bundle->reference_time.to_seconds());

        if (bundle->lidar)
            std::printf("  | LiDAR %u pts", static_cast<unsigned>(bundle->lidar->points.size()));
        if (bundle->imu)
            std::printf("  | IMU a=(%.2f,%.2f,%.2f)",
                        bundle->imu->accel[0], bundle->imu->accel[1], bundle->imu->accel[2]);
        if (bundle->camera)
            std::printf("  | Cam %ux%u seq=%u",
                        bundle->camera->width, bundle->camera->height,
                        bundle->camera->sequence_number);

        std::putchar('\n');
    });

    Status s = device.connect();
    if (s != Status::OK) { std::fprintf(stderr, "connect: %s\n", status_string(s)); return 1; }

    s = device.start();
    if (s != Status::OK) { std::fprintf(stderr, "start: %s\n", status_string(s)); return 1; }

    std::puts("Streaming synced bundles for 3 seconds ...\n");
    std::this_thread::sleep_for(std::chrono::seconds(3));

    device.stop();
    device.disconnect();

    std::printf("\nTotal bundles: %llu\n",
                static_cast<unsigned long long>(bundle_count.load()));
    return 0;
}
