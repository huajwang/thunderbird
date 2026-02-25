// ─────────────────────────────────────────────────────────────────────────────
// Test — SyncEngine
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/sync_engine.h"

#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <thread>

int main() {
    using namespace thunderbird;

    std::atomic<int> bundle_count{0};
    std::shared_ptr<const SyncBundle> last_bundle;

    SyncConfig sc;
    sc.tolerance_ns    = 100'000'000; // 100 ms — generous for test
    sc.poll_interval_ms = 2;

    SyncEngine engine(sc);
    engine.set_callback([&](std::shared_ptr<const SyncBundle> b) {
        last_bundle = b;
        ++bundle_count;
    });
    engine.start();

    // Create matching-timestamp data
    Timestamp t = Timestamp::now();

    auto lidar = std::make_shared<LidarFrame>();
    lidar->timestamp = t;
    lidar->points.resize(100);

    auto imu = std::make_shared<ImuSample>();
    imu->timestamp = {t.nanoseconds + 1'000'000}; // +1 ms

    auto cam = std::make_shared<CameraFrame>();
    cam->timestamp = {t.nanoseconds + 5'000'000}; // +5 ms
    cam->width = 640; cam->height = 480;

    engine.feed_lidar(lidar);
    engine.feed_imu(imu);
    engine.feed_camera(cam);

    // Wait for the sync thread to process
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    engine.stop();

    assert(bundle_count >= 1);
    assert(last_bundle != nullptr);
    assert(last_bundle->lidar != nullptr);
    assert(last_bundle->camera != nullptr);
    // IMU should also be matched
    assert(last_bundle->imu != nullptr);

    std::printf("SyncEngine: produced %d bundle(s). ALL TESTS PASSED\n", bundle_count.load());
    return 0;
}
