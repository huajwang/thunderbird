// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Data Abstraction Layer tests
// ─────────────────────────────────────────────────────────────────────────────
//
// Validates:
//   1. SensorQueue  — push / pop / drop-oldest / blocking / stats
//   2. DataLayer    — ingestion, Pull API, Callback API, zero-copy images
//   3. PointXYZIT   — timestamp offset computation
//   4. Integration  — DeviceManager ↔ DataLayer end-to-end (simulated)
//
// ─────────────────────────────────────────────────────────────────────────────

#include "thunderbird/sensor_data.h"
#include "thunderbird/sensor_queue.h"
#include "thunderbird/data_layer.h"
#include "thunderbird/types.h"
#include "thunderbird/device_manager.h"

#include <cassert>
#include <cstdio>
#include <chrono>
#include <thread>
#include <atomic>
#include <vector>
#include <cmath>

// Namespace aliases to avoid ambiguity between thunderbird::LidarFrame and
// thunderbird::data::LidarFrame when both namespaces are in scope.
namespace tb  = thunderbird;
namespace tbd = thunderbird::data;

// ─── Helpers ────────────────────────────────────────────────────────────────

/// Create a synthetic internal LidarFrame with N points.
static std::shared_ptr<tb::LidarFrame> make_lidar(uint32_t seq, int n) {
    auto f = std::make_shared<tb::LidarFrame>();
    f->timestamp      = tb::Timestamp{static_cast<int64_t>(seq) * 100'000'000LL};
    f->host_timestamp  = tb::Timestamp::now();
    f->sequence_number = seq;
    f->points.resize(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        f->points[i].x         = static_cast<float>(i) * 0.1f;
        f->points[i].y         = static_cast<float>(i) * 0.2f;
        f->points[i].z         = static_cast<float>(i) * 0.3f;
        f->points[i].intensity = static_cast<float>(i % 256);
        f->points[i].ring      = static_cast<uint8_t>(i % 16);
    }
    return f;
}

/// Create a synthetic internal ImuSample.
static std::shared_ptr<tb::ImuSample> make_imu(int64_t ts_ns) {
    auto s = std::make_shared<tb::ImuSample>();
    s->timestamp      = tb::Timestamp{ts_ns};
    s->host_timestamp  = tb::Timestamp::now();
    s->accel           = {0.0f, 0.0f, 9.81f};
    s->gyro            = {0.01f, 0.02f, 0.03f};
    s->temperature     = 25.0f;
    return s;
}

/// Create a synthetic internal CameraFrame.
static std::shared_ptr<tb::CameraFrame> make_camera(uint32_t seq,
                                                              uint32_t w,
                                                              uint32_t h) {
    auto f = std::make_shared<tb::CameraFrame>();
    f->timestamp      = tb::Timestamp{static_cast<int64_t>(seq) * 33'333'333LL};
    f->host_timestamp  = tb::Timestamp::now();
    f->sequence_number = seq;
    f->width           = w;
    f->height          = h;
    f->format          = tb::PixelFormat::RGB8;
    f->stride          = w * 3;
    f->data.resize(static_cast<size_t>(w) * h * 3, 0xAB);
    return f;
}

// ─── Test: SensorQueue basic push/pop ───────────────────────────────────────

static void test_sensor_queue_basic() {
    std::puts("test_sensor_queue_basic ...");

    tb::SensorQueue<int> q(4);
    assert(q.empty());
    assert(q.size() == 0);

    q.push(10);
    q.push(20);
    q.push(30);
    assert(q.size() == 3);

    auto v1 = q.try_pop();
    assert(v1.has_value() && *v1 == 10);
    auto v2 = q.try_pop();
    assert(v2.has_value() && *v2 == 20);
    auto v3 = q.try_pop();
    assert(v3.has_value() && *v3 == 30);
    auto v4 = q.try_pop();
    assert(!v4.has_value());

    std::puts("  PASS");
}

// ─── Test: SensorQueue drop-oldest ──────────────────────────────────────────

static void test_sensor_queue_drop_oldest() {
    std::puts("test_sensor_queue_drop_oldest ...");

    tb::SensorQueue<int> q(3);  // capacity = 3
    q.push(1);
    q.push(2);
    q.push(3);
    assert(q.dropped() == 0);

    // This push should drop '1' (oldest).
    q.push(4);
    assert(q.dropped() == 1);
    assert(q.size() == 3);

    auto v = q.try_pop();
    assert(v.has_value() && *v == 2); // '1' was dropped

    q.push(5);
    q.push(6);
    // Queue: [3, 4, 5] → push 6 drops '3'
    assert(q.dropped() == 2);

    std::puts("  PASS");
}

// ─── Test: SensorQueue blocking pop ─────────────────────────────────────────

static void test_sensor_queue_blocking() {
    std::puts("test_sensor_queue_blocking ...");

    tb::SensorQueue<int> q(8);

    // Timeout when empty.
    auto none = q.pop_for(std::chrono::milliseconds(50));
    assert(!none.has_value());

    // Push from another thread, pop with blocking.
    std::thread producer([&q] {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        q.push(42);
    });

    auto val = q.pop_for(std::chrono::milliseconds(500));
    assert(val.has_value() && *val == 42);
    producer.join();

    std::puts("  PASS");
}

// ─── Test: DataLayer LiDAR ingestion + Pull API ────────────────────────────

static void test_data_layer_lidar_pull() {
    std::puts("test_data_layer_lidar_pull ...");

    tbd::DataLayer layer;

    auto raw = make_lidar(1, 100);
    layer.ingestLidar(raw);

    auto frame = layer.getNextLidarFrame();
    assert(frame.has_value());
    assert(frame->sequence == 1);
    assert(frame->timestamp_ns == 100'000'000LL);
    assert(frame->points.size() == 100);

    // Verify the zero-copy span accessor.
    auto span = frame->point_span();
    assert(span.size() == 100);
    assert(std::abs(span[0].x - 0.0f) < 1e-6f);
    assert(std::abs(span[1].x - 0.1f) < 1e-6f);

    // Verify per-point timestamp offsets.
    // First point should be 0, last should be ~100ms (10Hz default).
    assert(frame->points.front().timestamp_offset_ns == 0);
    assert(frame->points.back().timestamp_offset_ns > 0);
    // Last offset should be approximately 100,000,000 ns (within PoC tolerance).
    int32_t last_offset = frame->points.back().timestamp_offset_ns;
    assert(last_offset >= 99'000'000 && last_offset <= 101'000'000);

    // Queue should be empty now.
    assert(!layer.getNextLidarFrame().has_value());

    std::puts("  PASS");
}

// ─── Test: DataLayer IMU ingestion + Pull API ───────────────────────────────

static void test_data_layer_imu_pull() {
    std::puts("test_data_layer_imu_pull ...");

    tbd::DataLayer layer;

    for (int i = 0; i < 5; ++i) {
        layer.ingestImu(make_imu(static_cast<int64_t>(i) * 5'000'000LL));
    }

    int count = 0;
    while (auto f = layer.getNextImuFrame()) {
        assert(f->timestamp_ns == static_cast<int64_t>(count) * 5'000'000LL);
        assert(std::abs(f->accel[2] - 9.81f) < 1e-5f);
        assert(std::abs(f->gyro[0] - 0.01f)  < 1e-6f);
        ++count;
    }
    assert(count == 5);

    std::puts("  PASS");
}

// ─── Test: DataLayer Camera ingestion + zero-copy ───────────────────────────

static void test_data_layer_camera_zero_copy() {
    std::puts("test_data_layer_camera_zero_copy ...");

    tbd::DataLayer layer;

    auto raw = make_camera(7, 320, 240);
    const size_t expected_bytes = 320u * 240u * 3u;

    layer.ingestCamera(raw);

    auto frame = layer.getNextImageFrame();
    assert(frame.has_value());
    assert(frame->width  == 320);
    assert(frame->height == 240);
    assert(frame->sequence == 7);
    assert(frame->format == tbd::PixelFormat::RGB8);
    assert(frame->stride == 320 * 3);

    // pixels() span should be the right size.
    auto px = frame->pixels();
    assert(px.size() == expected_bytes);

    // Verify shared_ptr data is valid (pixel values).
    assert(px[0] == 0xAB);

    // expected_size() helper.
    assert(frame->expected_size() == expected_bytes);

    std::puts("  PASS");
}

// ─── Test: DataLayer Callback API ───────────────────────────────────────────

static void test_data_layer_callback() {
    std::puts("test_data_layer_callback ...");

    tbd::DataLayer layer;

    int lidar_fired = 0;
    int imu_fired   = 0;
    int cam_fired   = 0;

    layer.onLidarFrame([&](const tbd::LidarFrame& f) {
        (void)f;
        ++lidar_fired;
    });
    layer.onImuFrame([&](const tbd::ImuFrame& f) {
        (void)f;
        ++imu_fired;
    });
    layer.onImageFrame([&](const tbd::ImageFrame& f) {
        (void)f;
        ++cam_fired;
    });

    layer.ingestLidar(make_lidar(0, 10));
    layer.ingestLidar(make_lidar(1, 20));
    layer.ingestImu(make_imu(1000));
    layer.ingestCamera(make_camera(0, 64, 64));

    assert(lidar_fired == 2);
    assert(imu_fired   == 1);
    assert(cam_fired   == 1);

    // Data should also be in the queue (both paths work simultaneously).
    assert(layer.lidarQueued() == 2);
    assert(layer.imuQueued()   == 1);
    assert(layer.cameraQueued() == 1);

    std::puts("  PASS");
}

// ─── Test: DataLayer flush ──────────────────────────────────────────────────

static void test_data_layer_flush() {
    std::puts("test_data_layer_flush ...");

    tbd::DataLayer layer;
    for (int i = 0; i < 10; ++i) layer.ingestImu(make_imu(i * 1000));
    assert(layer.imuQueued() == 10);

    layer.flush();
    assert(layer.imuQueued() == 0);
    assert(!layer.getNextImuFrame().has_value());

    std::puts("  PASS");
}

// ─── Test: Integration — DeviceManager ↔ DataLayer (simulated) ──────────────

static void test_integration_device_manager() {
    std::puts("test_integration_device_manager ...");

    tb::DeviceConfig cfg;
    cfg.lidar_hz   = 10.0;
    cfg.imu_hz     = 200.0;
    cfg.camera_fps = 30.0;

    tb::DeviceManager dev(cfg);
    assert(dev.connect() == tb::Status::OK);
    assert(dev.start()   == tb::Status::OK);

    // Let the simulated drivers produce data for ~250 ms.
    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    dev.stop();

    auto& layer = dev.data_layer();

    // We should have received *some* frames from each sensor.
    int lidar_n = 0, imu_n = 0, cam_n = 0;
    while (layer.getNextLidarFrame())  ++lidar_n;
    while (layer.getNextImuFrame())    ++imu_n;
    while (layer.getNextImageFrame())  ++cam_n;

    std::printf("  Received: LiDAR=%d  IMU=%d  Camera=%d\n",
                lidar_n, imu_n, cam_n);

    // At 10 Hz for 250 ms we expect ~2 LiDAR frames (at least 1).
    assert(lidar_n >= 1);
    // At 200 Hz for 250 ms we expect ~50 IMU (at least 10).
    assert(imu_n >= 10);
    // At 30 Hz for 250 ms we expect ~7 camera (at least 2).
    assert(cam_n >= 2);

    dev.disconnect();

    std::puts("  PASS");
}

// ─── Test: PointXYZIT and span accessor ─────────────────────────────────────

static void test_point_xyzit() {
    std::puts("test_point_xyzit ...");

    tbd::PointXYZIT pt;
    pt.x = 1.0f; pt.y = 2.0f; pt.z = 3.0f;
    pt.intensity = 128.0f;
    pt.timestamp_offset_ns = 50'000;

    assert(std::abs(pt.x - 1.0f) < 1e-6f);
    assert(pt.timestamp_offset_ns == 50'000);

    // LidarFrame span accessor.
    tbd::LidarFrame frame;
    frame.points.push_back(pt);
    frame.points.push_back({4.0f, 5.0f, 6.0f, 200.0f, 100'000});

    auto sp = frame.point_span();
    assert(sp.size() == 2);
    assert(std::abs(sp[1].x - 4.0f) < 1e-6f);
    assert(sp[1].timestamp_offset_ns == 100'000);

    std::puts("  PASS");
}

// ─── Test: ImageFrame helpers ───────────────────────────────────────────────

static void test_image_frame_helpers() {
    std::puts("test_image_frame_helpers ...");

    tbd::ImageFrame f;
    f.width  = 640;
    f.height = 480;
    f.format = tbd::PixelFormat::RGB8;
    assert(f.expected_size() == 640u * 480u * 3u);

    f.format = tbd::PixelFormat::Mono8;
    assert(f.expected_size() == 640u * 480u * 1u);

    // pixels() on empty data returns empty span.
    assert(f.pixels().empty());

    // With data.
    auto buf = std::make_shared<const std::vector<uint8_t>>(100, 0x42);
    f.data = buf;
    assert(f.pixels().size() == 100);
    assert(f.pixels()[0] == 0x42);

    std::puts("  PASS");
}

// ─── Main ───────────────────────────────────────────────────────────────────

int main() {
    std::puts("====== Data Abstraction Layer Tests ======\n");

    // SensorQueue tests
    test_sensor_queue_basic();
    test_sensor_queue_drop_oldest();
    test_sensor_queue_blocking();

    // DataLayer tests
    test_data_layer_lidar_pull();
    test_data_layer_imu_pull();
    test_data_layer_camera_zero_copy();
    test_data_layer_callback();
    test_data_layer_flush();

    // Public type tests
    test_point_xyzit();
    test_image_frame_helpers();

    // Integration test
    test_integration_device_manager();

    std::puts("\n====== ALL TESTS PASSED ======");
    return 0;
}
