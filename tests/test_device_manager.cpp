// ─────────────────────────────────────────────────────────────────────────────
// Test — DeviceManager (end-to-end with simulated backends)
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/thunderbird.h"

#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <thread>

int main() {
    using namespace thunderbird;

    // ── Basic lifecycle ─────────────────────────────────────────────────────
    {
        DeviceManager dev;
        assert(!dev.is_connected());
        assert(!dev.is_streaming());

        Status s = dev.connect();
        (void)s;
        assert(s == Status::OK);
        assert(dev.is_connected());

        auto info = dev.device_info();
        assert(!info.serial_number.empty());

        s = dev.disconnect();
        assert(s == Status::OK);
        assert(!dev.is_connected());
    }

    // ── Streaming produces data ─────────────────────────────────────────────
    {
        std::atomic<uint64_t> lidar_n{0}, imu_n{0}, cam_n{0};

        DeviceConfig cfg;
        cfg.lidar_hz   = 20.0;
        cfg.imu_hz     = 100.0;
        cfg.camera_fps = 20.0;

        DeviceManager dev(cfg);
        dev.on_lidar([&](auto) { ++lidar_n; });
        dev.on_imu([&](auto) { ++imu_n; });
        dev.on_camera([&](auto) { ++cam_n; });

        assert(dev.connect() == Status::OK);
        assert(dev.start()   == Status::OK);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        dev.stop();
        dev.disconnect();

        // Should have received noticeable amount of data in 500 ms
        std::printf("  lidar=%llu  imu=%llu  cam=%llu\n",
                    static_cast<unsigned long long>(lidar_n.load()),
                    static_cast<unsigned long long>(imu_n.load()),
                    static_cast<unsigned long long>(cam_n.load()));
        assert(lidar_n > 0);
        assert(imu_n   > 0);
        assert(cam_n   > 0);
    }

    // ── Sync bundles produced ───────────────────────────────────────────────
    {
        std::atomic<uint64_t> bundles{0};

        DeviceConfig cfg;
        cfg.lidar_hz   = 10.0;
        cfg.imu_hz     = 100.0;
        cfg.camera_fps = 10.0;
        cfg.sync.tolerance_ns = 120'000'000; // 120 ms

        DeviceManager dev(cfg);
        dev.on_sync([&](auto) { ++bundles; });

        assert(dev.connect() == Status::OK);
        assert(dev.start()   == Status::OK);

        std::this_thread::sleep_for(std::chrono::seconds(1));

        dev.stop();
        dev.disconnect();

        std::printf("  sync bundles=%llu\n",
                    static_cast<unsigned long long>(bundles.load()));
        assert(bundles > 0);
    }

    std::puts("DeviceManager: ALL TESTS PASSED");
    return 0;
}
