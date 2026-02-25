// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Unit tests for Time Synchronization Layer
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/time_sync.h"

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>
#include <vector>

namespace tbd = thunderbird::data;

// ── Helpers ─────────────────────────────────────────────────────────────────

static tbd::LidarFrame make_lidar(int64_t ts_ns, int n_pts = 10) {
    tbd::LidarFrame f;
    f.timestamp_ns = ts_ns;
    f.sequence     = 0;
    for (int i = 0; i < n_pts; ++i)
        f.points.push_back({static_cast<float>(i), 0.f, 0.f, 1.0f, 0});
    return f;
}

static tbd::ImuFrame make_imu(int64_t ts_ns) {
    tbd::ImuFrame f;
    f.timestamp_ns = ts_ns;
    f.accel[0] = 0; f.accel[1] = 0; f.accel[2] = 9.81;
    f.gyro[0]  = 0; f.gyro[1]  = 0; f.gyro[2]  = 0;
    return f;
}

static tbd::ImageFrame make_camera(int64_t ts_ns) {
    tbd::ImageFrame f;
    f.timestamp_ns = ts_ns;
    f.width  = 2;
    f.height = 2;
    f.stride = 6;
    f.format = tbd::PixelFormat::RGB8;
    auto px  = std::make_shared<std::vector<uint8_t>>(12, 128);
    f.data   = px;
    return f;
}

// ── Test 1: basic assembly ──────────────────────────────────────────────────

static void test_basic_assembly() {
    std::printf("test_basic_assembly ... ");

    tbd::TimeSyncConfig cfg;
    cfg.camera_tolerance_ns = 50'000'000;  // 50 ms
    cfg.poll_interval_ms = 1;
    cfg.output_queue_depth = 64;

    tbd::TimeSyncEngine engine(cfg);

    // Feed: IMU at 5 ms intervals, camera near first LiDAR
    for (int i = 0; i < 20; ++i)
        engine.feedImu(make_imu(i * 5'000'000));    // 0,5,10,...95 ms

    engine.feedCamera(make_camera(2'000'000));       // 2 ms
    engine.feedCamera(make_camera(102'000'000));     // 102 ms

    engine.feedLidar(make_lidar(0));                 // epoch 0
    engine.feedLidar(make_lidar(100'000'000));       // epoch 100 ms

    engine.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    engine.stop();

    // We should get at least 2 SyncedFrames.
    auto sf1 = engine.getNextSyncedFrame();
    assert(sf1.has_value());
    assert(sf1->lidar.timestamp_ns == 0);
    // Camera at 2 ms matches lidar at 0 ms  (diff = 2 ms < 50 ms tolerance).
    assert(sf1->camera.has_value());
    assert(sf1->camera->timestamp_ns == 2'000'000);
    // First frame may have empty imu_block (no prev_lidar_ts).
    // The first assembled frame: imu_block contains samples in (MIN, 0].
    assert(!sf1->imu_block.empty());
    assert(sf1->imu_block.back().timestamp_ns <= 0);

    auto sf2 = engine.getNextSyncedFrame();
    assert(sf2.has_value());
    assert(sf2->lidar.timestamp_ns == 100'000'000);
    // Camera at 102 ms matches lidar at 100 ms.
    assert(sf2->camera.has_value());
    assert(sf2->camera->timestamp_ns == 102'000'000);
    // IMU block: samples in (0, 100 ms] → 5,10,...,95 = 19 samples
    // (0 ms was consumed by the first bundle as <= 0)
    assert(!sf2->imu_block.empty());
    for (auto& imu : sf2->imu_block) {
        assert(imu.timestamp_ns > 0);
        assert(imu.timestamp_ns <= 100'000'000);
    }

    std::printf("PASS\n");
}

// ── Test 2: camera miss (outside tolerance) ────────────────────────────────

static void test_camera_miss() {
    std::printf("test_camera_miss ... ");

    tbd::TimeSyncConfig cfg;
    cfg.camera_tolerance_ns = 10'000'000;  // 10 ms
    cfg.poll_interval_ms = 1;

    tbd::TimeSyncEngine engine(cfg);

    // Camera 50 ms away from lidar → outside 10 ms tolerance.
    engine.feedCamera(make_camera(50'000'000));
    engine.feedLidar(make_lidar(0));

    engine.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    engine.stop();

    auto sf = engine.getNextSyncedFrame();
    assert(sf.has_value());
    assert(!sf->camera.has_value());   // should be missing
    assert(sf->sync_quality == 0.0f);

    auto st = engine.stats();
    assert(st.camera_misses >= 1);

    std::printf("PASS\n");
}

// ── Test 3: IMU block correctness ──────────────────────────────────────────

static void test_imu_block() {
    std::printf("test_imu_block ... ");

    tbd::TimeSyncConfig cfg;
    cfg.camera_tolerance_ns = 100'000'000;
    cfg.poll_interval_ms = 1;

    tbd::TimeSyncEngine engine(cfg);

    // 40 IMU samples at 2.5 ms intervals → 0..97.5 ms
    for (int i = 0; i < 40; ++i)
        engine.feedImu(make_imu(i * 2'500'000));

    engine.feedCamera(make_camera(0));
    engine.feedCamera(make_camera(50'000'000));

    engine.feedLidar(make_lidar(0));
    engine.feedLidar(make_lidar(50'000'000));

    engine.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    engine.stop();

    auto sf1 = engine.getNextSyncedFrame();
    assert(sf1.has_value());

    auto sf2 = engine.getNextSyncedFrame();
    assert(sf2.has_value());
    assert(sf2->lidar.timestamp_ns == 50'000'000);

    // IMU block for epoch 50 ms: samples in (0, 50'000'000]
    // That's 2.5, 5.0, ... 50.0 ms → 20 samples
    assert(sf2->imu_block.size() == 20);

    for (auto& imu : sf2->imu_block) {
        assert(imu.timestamp_ns > 0);
        assert(imu.timestamp_ns <= 50'000'000);
    }

    std::printf("PASS\n");
}

// ── Test 4: sync quality scoring ────────────────────────────────────────────

static void test_sync_quality() {
    std::printf("test_sync_quality ... ");

    tbd::TimeSyncConfig cfg;
    cfg.camera_tolerance_ns = 100'000'000;  // 100 ms
    cfg.poll_interval_ms = 1;

    tbd::TimeSyncEngine engine(cfg);

    // Camera exactly at lidar time → quality should be ~1.0
    engine.feedCamera(make_camera(0));
    engine.feedLidar(make_lidar(0));

    engine.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    engine.stop();

    auto sf = engine.getNextSyncedFrame();
    assert(sf.has_value());
    assert(sf->sync_quality >= 0.99f);  // perfect match

    std::printf("PASS\n");
}

// ── Test 5: callback delivery ───────────────────────────────────────────────

static void test_callback() {
    std::printf("test_callback ... ");

    tbd::TimeSyncConfig cfg;
    cfg.camera_tolerance_ns = 100'000'000;
    cfg.poll_interval_ms = 1;

    tbd::TimeSyncEngine engine(cfg);

    int callback_count = 0;
    engine.onSyncedFrame([&](const tbd::SyncedFrame& sf) {
        ++callback_count;
        (void)sf;
    });

    engine.feedCamera(make_camera(0));
    engine.feedCamera(make_camera(100'000'000));
    engine.feedLidar(make_lidar(0));
    engine.feedLidar(make_lidar(100'000'000));

    engine.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    engine.stop();

    assert(callback_count == 2);

    std::printf("PASS\n");
}

// ── Test 6: statistics tracking ─────────────────────────────────────────────

static void test_stats() {
    std::printf("test_stats ... ");

    tbd::TimeSyncConfig cfg;
    cfg.camera_tolerance_ns = 10'000'000;
    cfg.poll_interval_ms = 1;

    tbd::TimeSyncEngine engine(cfg);

    // First lidar: camera miss.  Second: camera hit.
    engine.feedLidar(make_lidar(0));
    engine.feedLidar(make_lidar(100'000'000));

    engine.feedCamera(make_camera(100'000'000));  // exact match for second

    engine.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    engine.stop();

    auto st = engine.stats();
    assert(st.frames_produced == 2);
    assert(st.camera_misses == 1);   // first had no camera
    assert(engine.framesProduced() == 2);

    std::printf("PASS\n");
}

// ── Test 7: flush clears state ──────────────────────────────────────────────

static void test_flush() {
    std::printf("test_flush ... ");

    tbd::TimeSyncConfig cfg;
    cfg.poll_interval_ms = 1;

    tbd::TimeSyncEngine engine(cfg);

    engine.feedLidar(make_lidar(0));
    engine.feedImu(make_imu(0));
    engine.feedCamera(make_camera(0));

    engine.flush();

    engine.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    engine.stop();

    // After flush, nothing should be produced from old data.
    auto sf = engine.getNextSyncedFrame();
    assert(!sf.has_value());

    std::printf("PASS\n");
}

// ── Test 8: drift estimation ────────────────────────────────────────────────

static void test_drift_estimation() {
    std::printf("test_drift_estimation ... ");

    tbd::TimeSyncConfig cfg;
    cfg.camera_tolerance_ns = 1'000'000'000;  // very wide
    cfg.drift_window = 10;
    cfg.drift_warn_ns_per_sec = 500'000;  // 0.5 ms/s
    cfg.poll_interval_ms = 1;

    tbd::TimeSyncEngine engine(cfg);

    bool drift_warned = false;
    engine.onDriftWarning([&](double d) {
        drift_warned = true;
        (void)d;
    });

    // Feed lidar at 100 ms intervals, camera with increasing drift
    //   lidar: 0, 100, 200, ... 900 ms
    //   camera: 0+0, 100+1, 200+2, ... 900+9 ms  (1 ms drift per 100 ms = 10 ms/s)
    for (int i = 0; i < 10; ++i) {
        int64_t lidar_ts  = static_cast<int64_t>(i) * 100'000'000;
        int64_t camera_ts = lidar_ts + static_cast<int64_t>(i) * 1'000'000;
        engine.feedLidar(make_lidar(lidar_ts));
        engine.feedCamera(make_camera(camera_ts));
    }

    engine.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    engine.stop();

    auto st = engine.stats();
    assert(st.frames_produced == 10);
    // drift should be approximately 10 ms/s = 10'000'000 ns/s
    // Check it's at least 5'000'000 ns/s (allowing estimation noise)
    assert(std::abs(st.drift_ns_per_sec) > 5'000'000);
    assert(drift_warned);

    std::printf("PASS\n");
}

// ── Test 9: blocking pull API ───────────────────────────────────────────────

static void test_blocking_pull() {
    std::printf("test_blocking_pull ... ");

    tbd::TimeSyncConfig cfg;
    cfg.camera_tolerance_ns = 100'000'000;
    cfg.poll_interval_ms = 1;

    tbd::TimeSyncEngine engine(cfg);
    engine.start();

    // Feed from a background thread after a short delay.
    std::thread feeder([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        engine.feedCamera(make_camera(0));
        engine.feedLidar(make_lidar(0));
    });

    auto sf = engine.waitForSyncedFrame(std::chrono::milliseconds(200));
    assert(sf.has_value());
    assert(sf->lidar.timestamp_ns == 0);

    feeder.join();
    engine.stop();

    std::printf("PASS\n");
}

// ── Main ────────────────────────────────────────────────────────────────────

int main() {
    std::printf("=== Thunderbird SDK — Time Sync Layer Tests ===\n\n");

    test_basic_assembly();
    test_camera_miss();
    test_imu_block();
    test_sync_quality();
    test_callback();
    test_stats();
    test_flush();
    test_drift_estimation();
    test_blocking_pull();

    std::printf("\nAll 9 tests passed.\n");
    return 0;
}
