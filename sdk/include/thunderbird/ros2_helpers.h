// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — ROS2 Conversion Helpers (no rclcpp dependency)
// ─────────────────────────────────────────────────────────────────────────────
//
// Pure utility functions that translate SDK public types into the numeric
// fields needed by ROS2 messages.  These live in the core SDK include tree
// (not in ros2_bridge/) so they can be:
//
//   1. Unit-tested without installing ROS2.
//   2. Reused by any ROS2 node, custom bridge, or log-replay tool.
//
// None of these functions reference rclcpp or *_msgs — they operate solely
// on plain C++ scalars and the thunderbird::data types from sensor_data.h.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/sensor_data.h"

#include <array>
#include <cstdint>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

namespace thunderbird::ros2_helpers {

// ─── Timestamp helpers ──────────────────────────────────────────────────────

/// Split a hardware nanosecond timestamp into {seconds, nanoseconds} as
/// required by builtin_interfaces/msg/Time.
struct StampParts {
    int32_t  sec{0};
    uint32_t nanosec{0};
};

[[nodiscard]] constexpr StampParts to_stamp(int64_t timestamp_ns) noexcept {
    StampParts s;
    s.sec     = static_cast<int32_t>(timestamp_ns / 1'000'000'000LL);
    s.nanosec = static_cast<uint32_t>(timestamp_ns % 1'000'000'000LL);
    return s;
}

// ─── PixelFormat ↔ ROS encoding string ──────────────────────────────────────

/// Map SDK PixelFormat to the ROS2 `sensor_msgs/Image` encoding string.
///
///   | SDK PixelFormat | ROS encoding |
///   |-----------------|--------------|
///   | Mono8           | "mono8"      |
///   | RGB8            | "rgb8"       |
///   | BGR8            | "bgr8"       |
///   | YUYV            | "yuv422_yuy2"|
///   | NV12            | "nv12"       |
///
/// Returns "rgb8" as a safe default for unknown formats.
[[nodiscard]] inline std::string pixel_format_to_encoding(
    data::PixelFormat fmt) noexcept
{
    switch (fmt) {
        case data::PixelFormat::Mono8: return "mono8";
        case data::PixelFormat::RGB8:  return "rgb8";
        case data::PixelFormat::BGR8:  return "bgr8";
        case data::PixelFormat::YUYV:  return "yuv422_yuy2";
        case data::PixelFormat::NV12:  return "nv12";
    }
    return "rgb8";
}

// ─── PointCloud2 field descriptors ──────────────────────────────────────────
//
// ROS2 sensor_msgs/PointCloud2 uses a flat byte buffer plus a list of
// "field descriptors" (name, offset, datatype, count).  We pre-compute
// these so the bridge can populate them quickly.

/// ROS2 PointField datatype constants (same values as sensor_msgs/PointField).
enum class PointFieldType : uint8_t {
    FLOAT32 = 7,
    FLOAT64 = 8,
    INT32   = 5,
    UINT32  = 6,
};

/// A single field descriptor matching sensor_msgs/msg/PointField layout.
struct FieldDescriptor {
    std::string    name;
    uint32_t       offset{0};
    PointFieldType datatype{PointFieldType::FLOAT32};
    uint32_t       count{1};
};

/// Returns the PointCloud2 field descriptors for our LiDAR point layout:
///   x (float32), y (float32), z (float32), intensity (float32),
///   timestamp_offset_ns (int32).
///
/// Total point_step = 20 bytes.
[[nodiscard]] inline std::vector<FieldDescriptor> lidar_point_fields() {
    return {
        {"x",                     0, PointFieldType::FLOAT32, 1},
        {"y",                     4, PointFieldType::FLOAT32, 1},
        {"z",                     8, PointFieldType::FLOAT32, 1},
        {"intensity",            12, PointFieldType::FLOAT32, 1},
        {"timestamp_offset_ns",  16, PointFieldType::INT32,   1},
    };
}

/// Bytes per point in our PointCloud2 layout (4+4+4+4+4 = 20).
inline constexpr uint32_t kPointStep = 20;

/// Serialise a vector of PointXYZIT into a flat byte buffer suitable for
/// PointCloud2::data.  The caller owns the returned vector.
///
/// Layout per point (20 bytes, little-endian):
///   [ x:f32 | y:f32 | z:f32 | intensity:f32 | offset_ns:i32 ]
[[nodiscard]] inline std::vector<uint8_t> serialise_points(
    const std::vector<data::PointXYZIT>& pts)
{
    const size_t n = pts.size();
    std::vector<uint8_t> buf(n * kPointStep);
    uint8_t* dst = buf.data();
    for (const auto& p : pts) {
        std::memcpy(dst +  0, &p.x,                     4);
        std::memcpy(dst +  4, &p.y,                     4);
        std::memcpy(dst +  8, &p.z,                     4);
        std::memcpy(dst + 12, &p.intensity,              4);
        std::memcpy(dst + 16, &p.timestamp_offset_ns,   4);
        dst += kPointStep;
    }
    return buf;
}

// ─── QoS constants ──────────────────────────────────────────────────────────
//
// ROS2 QoS "sensor data" profile: BEST_EFFORT reliability + small depth.
// We define the constants here so they're debuggable without rclcpp.

/// Default queue depth for sensor publishers.
inline constexpr size_t kSensorQosDepth = 5;

// ─── Frame-ID constants ────────────────────────────────────────────────────

inline constexpr std::string_view kLidarFrameId  = "thunderbird_lidar";
inline constexpr std::string_view kImuFrameId    = "thunderbird_imu";
inline constexpr std::string_view kCameraFrameId = "thunderbird_camera";
inline constexpr std::string_view kSyncFrameId   = "thunderbird_sync";

} // namespace thunderbird::ros2_helpers
