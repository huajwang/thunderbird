// ─────────────────────────────────────────────────────────────────────────────
// Time Synchronization Layer demo — Pull + Callback APIs for SyncedFrame
// ─────────────────────────────────────────────────────────────────────────────
//
// Build:  cmake --build build
// Run:    ./build/examples/sync_layer_demo
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/device_manager.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>

namespace tbd = thunderbird::data;

int main() {
    // ── Configure ───────────────────────────────────────────────────────
    thunderbird::DeviceConfig cfg;
    cfg.lidar_hz    = 10.0;
    cfg.imu_hz      = 200.0;
    cfg.camera_fps  = 30.0;
    cfg.camera_width  = 320;
    cfg.camera_height = 240;

    // Time-sync tuning
    cfg.time_sync.camera_tolerance_ns = 60'000'000;  // 60 ms
    cfg.time_sync.poll_interval_ms    = 1;

    thunderbird::DeviceManager dev(cfg);

    // ── Callback API ────────────────────────────────────────────────────
    std::atomic<int> cb_count{0};

    dev.on_synced_frame([&](const tbd::SyncedFrame& sf) {
        int n = ++cb_count;
        if (n <= 3 || n % 5 == 0) {
            std::printf("[callback] SyncedFrame #%llu  quality=%.2f  "
                        "imu_block=%zu  camera=%s  offset=%+lld ns\n",
                        static_cast<unsigned long long>(sf.sequence),
                        sf.sync_quality,
                        sf.imu_block.size(),
                        sf.camera ? "YES" : "NO ",
                        static_cast<long long>(sf.lidar_camera_offset_ns));
        }
    });

    // Drift warning
    dev.time_sync().onDriftWarning([](double drift) {
        std::printf("[drift-warning] clock drift = %.1f ns/s\n", drift);
    });

    // ── Connect & start ─────────────────────────────────────────────────
    auto s = dev.connect();
    if (s != thunderbird::Status::OK) {
        std::printf("connect failed: %d\n", static_cast<int>(s));
        return 1;
    }
    s = dev.start();
    if (s != thunderbird::Status::OK) {
        std::printf("start failed: %d\n", static_cast<int>(s));
        return 1;
    }

    std::printf("=== Streaming for 2 seconds ===\n");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // ── Pull API ────────────────────────────────────────────────────────
    std::printf("\n--- Pull API drain ---\n");
    int pull_count = 0;
    while (auto sf = dev.time_sync().getNextSyncedFrame()) {
        ++pull_count;
        if (pull_count <= 3) {
            std::printf("[pull] SyncedFrame #%llu  lidar_ts=%lld  "
                        "imu_block=%zu  camera=%s\n",
                        static_cast<unsigned long long>(sf->sequence),
                        static_cast<long long>(sf->lidar.timestamp_ns),
                        sf->imu_block.size(),
                        sf->camera ? "YES" : "NO ");
        }
    }
    std::printf("[pull] drained %d buffered frames\n", pull_count);

    // ── Stats ───────────────────────────────────────────────────────────
    auto st = dev.time_sync().stats();
    std::printf("\n--- Sync Stats ---\n"
                "  frames_produced: %llu\n"
                "  camera_misses:   %llu\n"
                "  imu_gaps:        %llu\n"
                "  mean_offset_ns:  %.0f\n"
                "  drift_ns/s:      %.1f\n",
                static_cast<unsigned long long>(st.frames_produced),
                static_cast<unsigned long long>(st.camera_misses),
                static_cast<unsigned long long>(st.imu_gaps),
                st.mean_offset_ns,
                st.drift_ns_per_sec);

    dev.stop();
    dev.disconnect();
    std::printf("\nDone. Callback delivered %d SyncedFrames.\n", cb_count.load());
    return 0;
}
