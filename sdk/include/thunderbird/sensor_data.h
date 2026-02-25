// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Public sensor data structures (Data Abstraction Layer)
// ─────────────────────────────────────────────────────────────────────────────
//
// These are the **developer-facing** types returned by the Pull API and
// delivered through the Callback API.  They are intentionally separate from
// the internal wire-protocol types (protocol.h) and the legacy types
// (types.h) so that the public API surface can evolve independently of
// the on-wire representation.
//
// Design principles:
//   • Timestamps are always **int64_t nanoseconds in the hardware clock
//     domain** — no conversions, no abstractions.  The user can convert
//     or rebase as needed.
//   • LiDAR points carry a *timestamp offset* relative to the frame
//     timestamp, allowing per-point motion compensation on the host.
//   • ImageFrame stores pixel data in a shared_ptr<vector> so that the
//     Pull API and Callback API can share the same allocation without
//     copying.  A `std::span` accessor gives zero-copy read access.
//   • All structs are value types (movable, copyable) and trivially
//     serialisable for downstream consumers.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <cstdint>
#include <cstddef>
#include <array>
#include <vector>
#include <memory>
#include <span>

namespace thunderbird::data {

// ─── LiDAR ──────────────────────────────────────────────────────────────────

/// A single LiDAR return with per-point timing information.
///
/// `timestamp_offset_ns` is the delta from the parent LidarFrame::timestamp_ns
/// to the acquisition moment of this point.  This enables downstream
/// motion-compensation: absolute_time = frame.timestamp_ns + pt.timestamp_offset_ns
struct PointXYZIT {
    float   x{0};                   ///< metres
    float   y{0};                   ///< metres
    float   z{0};                   ///< metres
    float   intensity{0};           ///< normalised 0–255
    int32_t timestamp_offset_ns{0}; ///< ns relative to frame timestamp
};

/// One complete (or partial) LiDAR sweep.
///
/// `points` owns the data; call `point_span()` for a zero-copy view.
struct LidarFrame {
    int64_t  timestamp_ns{0};       ///< hardware frame timestamp (ns)
    uint32_t sequence{0};           ///< monotonic frame counter

    std::vector<PointXYZIT> points; ///< array of returns

    /// Zero-copy span over the point array.
    [[nodiscard]] std::span<const PointXYZIT> point_span() const noexcept {
        return {points.data(), points.size()};
    }
};

// ─── IMU ────────────────────────────────────────────────────────────────────

/// A single IMU measurement (6-DOF).
struct ImuFrame {
    int64_t timestamp_ns{0};                ///< hardware timestamp (ns)

    std::array<float, 3> accel{};           ///< m/s² (x, y, z)
    std::array<float, 3> gyro{};            ///< rad/s (x, y, z)
};

// ─── Camera ─────────────────────────────────────────────────────────────────

/// Pixel encoding (matches the wire-protocol PixelFormat).
enum class PixelFormat : uint8_t {
    Mono8  = 0,
    RGB8   = 1,
    BGR8   = 2,
    YUYV   = 3,
    NV12   = 4,
};

/// Returns bytes-per-pixel for a given format (packed; YUYV averages 2).
inline constexpr uint32_t bytes_per_pixel(PixelFormat fmt) noexcept {
    switch (fmt) {
        case PixelFormat::Mono8: return 1;
        case PixelFormat::RGB8:  return 3;
        case PixelFormat::BGR8:  return 3;
        case PixelFormat::YUYV:  return 2;
        case PixelFormat::NV12:  return 1; // luma plane only; total is 1.5×
    }
    return 1;
}

/// A single camera image.
///
/// Pixel data is held through a `shared_ptr<vector<uint8_t>>` so the same
/// allocation can be referenced from the internal queue **and** the user's
/// callback / poll result without a copy.  Use `pixels()` for a lightweight
/// read-only span.
struct ImageFrame {
    int64_t  timestamp_ns{0};       ///< hardware timestamp (ns, exposure mid)
    uint32_t width{0};              ///< columns
    uint32_t height{0};             ///< rows
    uint32_t stride{0};             ///< bytes per row
    PixelFormat format{PixelFormat::RGB8};
    uint32_t sequence{0};           ///< monotonic frame counter

    /// Shared pixel buffer — zero-copy between producer and consumer.
    std::shared_ptr<const std::vector<uint8_t>> data;

    /// Zero-copy read-only span over the pixel buffer.
    [[nodiscard]] std::span<const uint8_t> pixels() const noexcept {
        if (data) return {data->data(), data->size()};
        return {};
    }

    /// Total expected byte count for this image (width × height × bpp).
    [[nodiscard]] size_t expected_size() const noexcept {
        return static_cast<size_t>(width) * height * bytes_per_pixel(format);
    }
};

} // namespace thunderbird::data
