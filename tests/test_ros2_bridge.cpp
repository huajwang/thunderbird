// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Unit tests for ROS2 conversion helpers
// ─────────────────────────────────────────────────────────────────────────────
//
// These tests exercise the ros2_helpers.h conversion utilities that translate
// SDK types into numeric fields suitable for ROS2 messages.  Crucially, they
// do NOT require rclcpp or any ROS2 installation — they compile and run with
// the standard SDK build (cmake, no colcon).
//
// This validates:
//   • Timestamp → {sec, nanosec} splitting
//   • PixelFormat → encoding string mapping
//   • PointCloud2 field descriptor generation
//   • Point serialisation into a flat byte buffer
//   • Frame-ID and QoS constants
//
// ─────────────────────────────────────────────────────────────────────────────

#include "thunderbird/ros2_helpers.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace rh = thunderbird::ros2_helpers;
namespace td = thunderbird::data;

// ── Test 1: timestamp conversion ────────────────────────────────────────────

static void test_to_stamp_basic() {
    std::printf("test_to_stamp_basic ... ");

    // 1.5 seconds = 1'500'000'000 ns
    constexpr int64_t ts = 1'500'000'000LL;
    auto s = rh::to_stamp(ts);
    assert(s.sec == 1);
    assert(s.nanosec == 500'000'000u);

    std::printf("PASS\n");
}

static void test_to_stamp_zero() {
    std::printf("test_to_stamp_zero ... ");

    auto s = rh::to_stamp(0);
    assert(s.sec == 0);
    assert(s.nanosec == 0u);

    std::printf("PASS\n");
}

static void test_to_stamp_large() {
    std::printf("test_to_stamp_large ... ");

    // 3600.123456789 seconds
    constexpr int64_t ts = 3600LL * 1'000'000'000LL + 123'456'789LL;
    auto s = rh::to_stamp(ts);
    assert(s.sec == 3600);
    assert(s.nanosec == 123'456'789u);

    std::printf("PASS\n");
}

static void test_to_stamp_exact_second() {
    std::printf("test_to_stamp_exact_second ... ");

    constexpr int64_t ts = 2'000'000'000LL;
    auto s = rh::to_stamp(ts);
    assert(s.sec == 2);
    assert(s.nanosec == 0u);

    std::printf("PASS\n");
}

// ── Test 2: pixel format encoding ───────────────────────────────────────────

static void test_pixel_format_encoding() {
    std::printf("test_pixel_format_encoding ... ");

    assert(rh::pixel_format_to_encoding(td::PixelFormat::Mono8) == "mono8");
    assert(rh::pixel_format_to_encoding(td::PixelFormat::RGB8)  == "rgb8");
    assert(rh::pixel_format_to_encoding(td::PixelFormat::BGR8)  == "bgr8");
    assert(rh::pixel_format_to_encoding(td::PixelFormat::YUYV)  == "yuv422_yuy2");
    assert(rh::pixel_format_to_encoding(td::PixelFormat::NV12)  == "nv12");

    std::printf("PASS\n");
}

// ── Test 3: PointCloud2 field descriptors ───────────────────────────────────

static void test_lidar_point_fields() {
    std::printf("test_lidar_point_fields ... ");

    auto fields = rh::lidar_point_fields();
    assert(fields.size() == 5);

    // x: offset 0, float32
    assert(fields[0].name == "x");
    assert(fields[0].offset == 0);
    assert(fields[0].datatype == rh::PointFieldType::FLOAT32);
    assert(fields[0].count == 1);

    // y: offset 4
    assert(fields[1].name == "y");
    assert(fields[1].offset == 4);

    // z: offset 8
    assert(fields[2].name == "z");
    assert(fields[2].offset == 8);

    // intensity: offset 12
    assert(fields[3].name == "intensity");
    assert(fields[3].offset == 12);
    assert(fields[3].datatype == rh::PointFieldType::FLOAT32);

    // timestamp_offset_ns: offset 16, int32
    assert(fields[4].name == "timestamp_offset_ns");
    assert(fields[4].offset == 16);
    assert(fields[4].datatype == rh::PointFieldType::INT32);

    // Total point step
    assert(rh::kPointStep == 20);

    std::printf("PASS\n");
}

// ── Test 4: point serialisation ─────────────────────────────────────────────

static void test_serialise_points_empty() {
    std::printf("test_serialise_points_empty ... ");

    std::vector<td::PointXYZIT> empty;
    auto buf = rh::serialise_points(empty);
    assert(buf.empty());

    std::printf("PASS\n");
}

static void test_serialise_points_one() {
    std::printf("test_serialise_points_one ... ");

    std::vector<td::PointXYZIT> pts = {
        {1.0f, 2.0f, 3.0f, 0.5f, 42}
    };

    auto buf = rh::serialise_points(pts);
    assert(buf.size() == 20);

    float x, y, z, intensity;
    int32_t offset;
    std::memcpy(&x,         buf.data() +  0, 4);
    std::memcpy(&y,         buf.data() +  4, 4);
    std::memcpy(&z,         buf.data() +  8, 4);
    std::memcpy(&intensity, buf.data() + 12, 4);
    std::memcpy(&offset,    buf.data() + 16, 4);

    assert(x == 1.0f);
    assert(y == 2.0f);
    assert(z == 3.0f);
    assert(intensity == 0.5f);
    assert(offset == 42);

    std::printf("PASS\n");
}

static void test_serialise_points_multiple() {
    std::printf("test_serialise_points_multiple ... ");

    std::vector<td::PointXYZIT> pts = {
        {0.0f, 0.0f, 0.0f, 0.0f, 0},
        {1.0f, 1.0f, 1.0f, 1.0f, 1000},
        {2.0f, 2.0f, 2.0f, 2.0f, 2000},
    };

    auto buf = rh::serialise_points(pts);
    assert(buf.size() == 60);  // 3 * 20

    // Check third point at offset 40
    float x;
    int32_t offset;
    std::memcpy(&x,      buf.data() + 40, 4);
    std::memcpy(&offset, buf.data() + 56, 4);
    assert(x == 2.0f);
    assert(offset == 2000);

    std::printf("PASS\n");
}

// ── Test 5: frame ID constants ──────────────────────────────────────────────

static void test_frame_id_constants() {
    std::printf("test_frame_id_constants ... ");

    assert(rh::kLidarFrameId  == "thunderbird_lidar");
    assert(rh::kImuFrameId    == "thunderbird_imu");
    assert(rh::kCameraFrameId == "thunderbird_camera");
    assert(rh::kSyncFrameId   == "thunderbird_sync");

    std::printf("PASS\n");
}

// ── Test 6: QoS depth constant ─────────────────────────────────────────────

static void test_qos_constant() {
    std::printf("test_qos_constant ... ");

    assert(rh::kSensorQosDepth == 5);

    std::printf("PASS\n");
}

// ── Test 7: roundtrip timestamp fidelity ────────────────────────────────────

static void test_timestamp_roundtrip() {
    std::printf("test_timestamp_roundtrip ... ");

    // Verify that sec * 1e9 + nanosec == original for various values.
    const int64_t test_values[] = {
        0,
        1,
        999'999'999,
        1'000'000'000,
        1'000'000'001,
        123'456'789'012'345LL,
    };

    for (auto ts : test_values) {
        auto s = rh::to_stamp(ts);
        int64_t reconstructed =
            static_cast<int64_t>(s.sec) * 1'000'000'000LL +
            static_cast<int64_t>(s.nanosec);
        (void)reconstructed;
        assert(reconstructed == ts);
    }

    std::printf("PASS\n");
}

// ── Main ────────────────────────────────────────────────────────────────────

int main() {
    std::printf("=== Thunderbird SDK — ROS2 Helpers Tests ===\n\n");

    test_to_stamp_basic();
    test_to_stamp_zero();
    test_to_stamp_large();
    test_to_stamp_exact_second();
    test_pixel_format_encoding();
    test_lidar_point_fields();
    test_serialise_points_empty();
    test_serialise_points_one();
    test_serialise_points_multiple();
    test_frame_id_constants();
    test_qos_constant();
    test_timestamp_roundtrip();

    std::printf("\nAll 12 tests passed.\n");
    return 0;
}
