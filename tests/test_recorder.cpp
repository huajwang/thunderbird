// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Recorder / Player unit tests
// ─────────────────────────────────────────────────────────────────────────────
//
// Tests cover:
//   1.  Binary format header validation (magic, version, device info)
//   2.  Round-trip: record N frames → play back → compare field-by-field
//   3.  Multi-sensor interleaved recording
//   4.  Empty recording / player on non-existent file
//   5.  Callback API delivery
//   6.  Playback statistics
//   7.  As-fast-as-possible playback (speed = 0)
//
// ─────────────────────────────────────────────────────────────────────────────

#include "thunderbird/recorder.h"
#include "thunderbird/player.h"
#include "thunderbird/recording_format.h"
#include "thunderbird/sensor_data.h"

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace tbd = thunderbird::data;
namespace rec = thunderbird::recording;

// Evaluate expression even under NDEBUG (prevents optimizing away side-effects).
#define VERIFY(expr)      do { bool _v = static_cast<bool>(expr); assert(_v); (void)_v; } while(0)
#define VERIFY_FALSE(expr) do { bool _v = static_cast<bool>(expr); assert(!_v); (void)_v; } while(0)

// ─── Helpers ────────────────────────────────────────────────────────────────

static const char* kTestFile = "test_recording.tbrec";

static void cleanup() { std::remove(kTestFile); }

static tbd::LidarFrame make_lidar(int64_t ts, uint32_t seq, int npts) {
    tbd::LidarFrame f;
    f.timestamp_ns = ts;
    f.sequence     = seq;
    f.points.resize(static_cast<std::size_t>(npts));
    for (int i = 0; i < npts; ++i) {
        auto& p     = f.points[static_cast<std::size_t>(i)];
        p.x         = static_cast<float>(i) * 0.1f;
        p.y         = static_cast<float>(i) * 0.2f;
        p.z         = static_cast<float>(i) * 0.05f;
        p.intensity = static_cast<float>(i % 256);
        p.timestamp_offset_ns = i * 100;
    }
    return f;
}

static tbd::ImuFrame make_imu(int64_t ts) {
    tbd::ImuFrame f;
    f.timestamp_ns = ts;
    f.accel = {0.01f, 0.02f, 9.81f};
    f.gyro  = {0.001f, -0.002f, 0.003f};
    return f;
}

static tbd::ImageFrame make_camera(int64_t ts, uint32_t seq,
                                   uint32_t w = 64, uint32_t h = 48)
{
    tbd::ImageFrame f;
    f.timestamp_ns = ts;
    f.width    = w;
    f.height   = h;
    f.stride   = w * 3;
    f.format   = tbd::PixelFormat::RGB8;
    f.sequence = seq;
    auto buf = std::make_shared<std::vector<uint8_t>>(
        static_cast<std::size_t>(f.stride) * h);
    for (std::size_t i = 0; i < buf->size(); ++i)
        (*buf)[i] = static_cast<uint8_t>((ts + i) & 0xFF);
    f.data = std::move(buf);
    return f;
}

// Approximate float comparison
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
static bool feq(float a, float b, float eps = 1e-6f) {
    return std::fabs(a - b) < eps;
}
#pragma GCC diagnostic pop

// ─── Tests ──────────────────────────────────────────────────────────────────

static int tests_run    = 0;
static int tests_passed = 0;

#define RUN_TEST(fn) do { \
    ++tests_run; \
    std::printf("  [%d] %-50s ", tests_run, #fn); \
    cleanup(); \
    fn(); \
    cleanup(); \
    ++tests_passed; \
    std::printf("PASS\n"); \
} while(0)

// 1. File header validation
static void test_file_header() {
    tbd::RecorderDeviceInfo info{
        .serial_number   = "SN-1234",
        .firmware_version = "2.0.1",
        .model_name       = "TestModel"
    };
    tbd::Recorder recorder(kTestFile, info);
    VERIFY(recorder.start());
    recorder.recordImuFrame(make_imu(5'000'000));
    recorder.stop();

    // Read raw header
    FILE* fp = std::fopen(kTestFile, "rb");
    assert(fp);
    rec::FileHeader fh{};
    size_t nread = std::fread(&fh, sizeof(fh), 1, fp);
    assert(nread == 1);
    (void)nread;
    std::fclose(fp);

    assert(std::memcmp(fh.magic, rec::kFileMagic, 8) == 0);
    assert(fh.version == rec::kFormatVersion);
    assert(fh.flags == 0);
    assert(std::string(fh.serial_number) == "SN-1234");
    assert(std::string(fh.firmware_version) == "2.0.1");
    assert(std::string(fh.model_name) == "TestModel");
    assert(fh.total_records == 1);
    assert(fh.start_timestamp_ns == 5'000'000);
    assert(fh.stop_timestamp_ns == 5'000'000);
    (void)fh;
}

// 2. LiDAR round-trip
static void test_lidar_roundtrip() {
    {
        tbd::Recorder rec(kTestFile);
        rec.start();
        rec.recordLidarFrame(make_lidar(100'000, 7, 50));
        rec.recordLidarFrame(make_lidar(200'000, 8, 30));
        rec.stop();
        assert(rec.stats().lidar_frames == 2);
    }
    {
        tbd::Player player(kTestFile, {.playback_speed = 0.0});
        VERIFY(player.start());

        while (!player.finished())
            std::this_thread::sleep_for(std::chrono::milliseconds(5));

        // Drain
        auto f1 = player.getNextLidarFrame();
        auto f2 = player.getNextLidarFrame();
        auto f3 = player.getNextLidarFrame();

        assert(f1.has_value());
        assert(f2.has_value());
        assert(!f3.has_value());

        assert(f1->timestamp_ns == 100'000);
        assert(f1->sequence == 7);
        assert(f1->points.size() == 50);
        assert(feq(f1->points[0].x, 0.0f));
        assert(feq(f1->points[1].x, 0.1f));

        assert(f2->timestamp_ns == 200'000);
        assert(f2->sequence == 8);
        assert(f2->points.size() == 30);

        player.stop();
    }
}

// 3. IMU round-trip
static void test_imu_roundtrip() {
    {
        tbd::Recorder rec(kTestFile);
        rec.start();
        for (int i = 0; i < 10; ++i)
            rec.recordImuFrame(make_imu(1'000'000 + i * 5'000));
        rec.stop();
        assert(rec.stats().imu_frames == 10);
    }
    {
        tbd::Player player(kTestFile, {.playback_speed = 0.0});
        player.start();
        while (!player.finished())
            std::this_thread::sleep_for(std::chrono::milliseconds(5));

        int count = 0;
        while (auto f = player.getNextImuFrame()) {
            assert(f->timestamp_ns == 1'000'000 + count * 5'000);
            assert(feq(f->accel[0], 0.01f));
            assert(feq(f->accel[2], 9.81f));
            assert(feq(f->gyro[1], -0.002f));
            ++count;
        }
        assert(count == 10);
        player.stop();
    }
}

// 4. Camera round-trip (pixel data fidelity)
static void test_camera_roundtrip() {
    const auto orig = make_camera(42'000, 3, 80, 60);
    {
        tbd::Recorder rec(kTestFile);
        rec.start();
        rec.recordImageFrame(orig);
        rec.stop();
    }
    {
        tbd::Player player(kTestFile, {.playback_speed = 0.0});
        player.start();
        while (!player.finished())
            std::this_thread::sleep_for(std::chrono::milliseconds(5));

        auto f = player.getNextImageFrame();
        assert(f.has_value());
        assert(f->timestamp_ns == 42'000);
        assert(f->width == 80);
        assert(f->height == 60);
        assert(f->stride == 240);
        assert(f->format == tbd::PixelFormat::RGB8);
        assert(f->sequence == 3);
        assert(f->data);
        assert(f->data->size() == orig.data->size());
        assert(std::memcmp(f->data->data(), orig.data->data(),
                           orig.data->size()) == 0);
        player.stop();
    }
}

// 5. Interleaved multi-sensor recording
static void test_interleaved() {
    {
        tbd::Recorder rec(kTestFile);
        rec.start();
        // Mirror real sensor pattern: LiDAR frame, then several IMU + camera
        for (int i = 0; i < 5; ++i) {
            int64_t base = 1'000'000 * (i + 1);
            rec.recordLidarFrame(make_lidar(base, static_cast<uint32_t>(i), 20));
            for (int j = 0; j < 4; ++j)
                rec.recordImuFrame(make_imu(base + j * 1'000));
            rec.recordImageFrame(make_camera(base + 500, static_cast<uint32_t>(i), 32, 24));
        }
        rec.stop();
        auto s = rec.stats();
        assert(s.lidar_frames == 5);
        assert(s.imu_frames == 20);
        assert(s.camera_frames == 5);
        (void)s;
    }
    {
        tbd::Player player(kTestFile, {.playback_speed = 0.0});
        player.start();
        while (!player.finished())
            std::this_thread::sleep_for(std::chrono::milliseconds(5));

        auto s = player.stats();
        assert(s.lidar_frames == 5);
        assert(s.imu_frames == 20);
        assert(s.camera_frames == 5);
        assert(s.total_records == 30);
        (void)s;
        player.stop();
    }
}

// 6. Empty recording
static void test_empty_recording() {
    {
        tbd::Recorder rec(kTestFile);
        rec.start();
        rec.stop();  // no frames written
    }
    {
        tbd::Player player(kTestFile, {.playback_speed = 0.0});
        VERIFY(player.start());
        while (!player.finished())
            std::this_thread::sleep_for(std::chrono::milliseconds(5));

        assert(!player.getNextLidarFrame().has_value());
        assert(!player.getNextImuFrame().has_value());
        assert(!player.getNextImageFrame().has_value());

        auto s = player.stats();
        assert(s.total_records == 0);
        (void)s;
        player.stop();
    }
}

// 7. Player rejects invalid file
static void test_invalid_file() {
    // Non-existent file
    tbd::Player p1("no_such_file.tbrec");
    VERIFY_FALSE(p1.start());

    // File with bad magic
    {
        FILE* fp = std::fopen(kTestFile, "wb");
        const char bad[8] = {'B','A','D','M','A','G','I','C'};
        std::fwrite(bad, 8, 1, fp);
        std::fclose(fp);
    }
    tbd::Player p2(kTestFile);
    VERIFY_FALSE(p2.start());
}

// 8. Callback API delivery
static void test_callback_api() {
    {
        tbd::Recorder rec(kTestFile);
        rec.start();
        rec.recordLidarFrame(make_lidar(1'000, 0, 10));
        rec.recordImuFrame(make_imu(2'000));
        rec.recordImageFrame(make_camera(3'000, 0));
        rec.stop();
    }
    {
        tbd::Player player(kTestFile, {.playback_speed = 0.0});

        int lidar_n = 0, imu_n = 0, cam_n = 0;
        int64_t got_lidar_ts = 0, got_imu_ts = 0, got_cam_ts = 0;

        player.onLidarFrame([&](const tbd::LidarFrame& f) {
            ++lidar_n;
            got_lidar_ts = f.timestamp_ns;
        });
        player.onImuFrame([&](const tbd::ImuFrame& f) {
            ++imu_n;
            got_imu_ts = f.timestamp_ns;
        });
        player.onImageFrame([&](const tbd::ImageFrame& f) {
            ++cam_n;
            got_cam_ts = f.timestamp_ns;
        });

        player.start();
        while (!player.finished())
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        player.stop();

        assert(lidar_n == 1);
        assert(imu_n == 1);
        assert(cam_n == 1);
        assert(got_lidar_ts == 1'000);
        assert(got_imu_ts == 2'000);
        assert(got_cam_ts == 3'000);
    }
}

// 9. Device info round-trip
static void test_device_info_roundtrip() {
    {
        tbd::Recorder rec(kTestFile,
                          {.serial_number   = "ABC-999",
                           .firmware_version = "3.2.1",
                           .model_name       = "SensorPro"});
        rec.start();
        rec.recordImuFrame(make_imu(1));
        rec.stop();
    }
    {
        tbd::Player player(kTestFile, {.playback_speed = 0.0});
        player.start();

        auto info = player.deviceInfo();
        assert(info.serial_number   == "ABC-999");
        assert(info.firmware_version == "3.2.1");
        assert(info.model_name       == "SensorPro");

        while (!player.finished())
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        player.stop();
    }
}

// 10. Recorder stats accumulation
static void test_recorder_stats() {
    tbd::Recorder rec(kTestFile);
    rec.start();
    for (int i = 0; i < 3; ++i)
        rec.recordLidarFrame(make_lidar(i * 1'000, static_cast<uint32_t>(i), 5));
    for (int i = 0; i < 7; ++i)
        rec.recordImuFrame(make_imu(i * 500));
    for (int i = 0; i < 2; ++i)
        rec.recordImageFrame(make_camera(i * 2'000, static_cast<uint32_t>(i)));
    rec.stop();

    auto s = rec.stats();
    assert(s.lidar_frames == 3);
    assert(s.imu_frames == 7);
    assert(s.camera_frames == 2);
    assert(s.total_bytes > 0);
    (void)s;
}

// 11. Double start / double stop (idempotent)
static void test_double_start_stop() {
    tbd::Recorder rec(kTestFile);
    VERIFY(rec.start());
    VERIFY_FALSE(rec.start());   // already recording → returns false
    rec.stop();
    rec.stop();             // double stop should be safe (no-op)

    // Same for player
    tbd::Player player(kTestFile, {.playback_speed = 0.0});
    VERIFY(player.start());
    VERIFY_FALSE(player.start());
    // Wait for completion
    while (!player.finished())
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    player.stop();
    player.stop();  // double stop safe
}

// ─── Main ───────────────────────────────────────────────────────────────────

int main() {
    std::printf("Thunderbird Recorder/Player Tests\n");
    std::printf("==================================\n");

    RUN_TEST(test_file_header);
    RUN_TEST(test_lidar_roundtrip);
    RUN_TEST(test_imu_roundtrip);
    RUN_TEST(test_camera_roundtrip);
    RUN_TEST(test_interleaved);
    RUN_TEST(test_empty_recording);
    RUN_TEST(test_invalid_file);
    RUN_TEST(test_callback_api);
    RUN_TEST(test_device_info_roundtrip);
    RUN_TEST(test_recorder_stats);
    RUN_TEST(test_double_start_stop);

    std::printf("\n%d / %d tests passed.\n", tests_passed, tests_run);
    cleanup();
    return (tests_passed == tests_run) ? 0 : 1;
}
