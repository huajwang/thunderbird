// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Core type definitions
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <array>
#include <chrono>
#include <functional>
#include <memory>

namespace thunderbird {

// ─── Timestamp ──────────────────────────────────────────────────────────────

/// Nanosecond-precision timestamp used throughout the SDK.
/// Epoch is configurable (defaults to system boot / PTP epoch).
struct Timestamp {
    int64_t nanoseconds{0};

    static Timestamp now() {
        using clock = std::chrono::steady_clock;
        auto d = clock::now().time_since_epoch();
        return {std::chrono::duration_cast<std::chrono::nanoseconds>(d).count()};
    }

    double to_seconds() const { return nanoseconds / 1.0e9; }

    Timestamp operator-(const Timestamp& o) const { return {nanoseconds - o.nanoseconds}; }
    Timestamp operator+(const Timestamp& o) const { return {nanoseconds + o.nanoseconds}; }
    bool operator<(const Timestamp& o)  const { return nanoseconds < o.nanoseconds; }
    bool operator<=(const Timestamp& o) const { return nanoseconds <= o.nanoseconds; }
    bool operator>(const Timestamp& o)  const { return nanoseconds > o.nanoseconds; }
    bool operator>=(const Timestamp& o) const { return nanoseconds >= o.nanoseconds; }
    bool operator==(const Timestamp& o) const { return nanoseconds == o.nanoseconds; }
};

// ─── Sensor identifiers ────────────────────────────────────────────────────

enum class SensorType : uint8_t {
    LiDAR  = 0,
    IMU    = 1,
    Camera = 2,
};

inline const char* sensor_type_name(SensorType t) {
    switch (t) {
        case SensorType::LiDAR:  return "LiDAR";
        case SensorType::IMU:    return "IMU";
        case SensorType::Camera: return "Camera";
    }
    return "Unknown";
}

// ─── LiDAR data ────────────────────────────────────────────────────────────

struct LidarPoint {
    float x, y, z;          // metres
    float intensity;         // 0–255 normalised
    uint8_t ring;            // laser ring / channel id
};

struct LidarFrame {
    Timestamp   timestamp;          // hardware timestamp of scan start
    Timestamp   host_timestamp;     // host arrival time
    uint32_t    sequence_number{0};
    std::vector<LidarPoint> points;
};

// ─── IMU data ──────────────────────────────────────────────────────────────

struct ImuSample {
    Timestamp timestamp;            // hardware timestamp
    Timestamp host_timestamp;

    std::array<float, 3> accel;     // m/s²  (x, y, z)
    std::array<float, 3> gyro;      // rad/s (x, y, z)
    float temperature;              // °C
};

// ─── Camera data ───────────────────────────────────────────────────────────

enum class PixelFormat : uint8_t {
    Mono8    = 0,
    RGB8     = 1,
    BGR8     = 2,
    YUYV     = 3,
    NV12     = 4,
};

struct CameraFrame {
    Timestamp   timestamp;          // hardware timestamp (exposure midpoint)
    Timestamp   host_timestamp;
    uint32_t    sequence_number{0};

    uint32_t    width{0};
    uint32_t    height{0};
    PixelFormat format{PixelFormat::RGB8};
    uint32_t    stride{0};          // bytes per row
    std::vector<uint8_t> data;
};

// ─── Synchronized bundle ───────────────────────────────────────────────────

/// A time-aligned bundle produced by the SyncEngine.
struct SyncBundle {
    Timestamp                         reference_time;   // aligned "midpoint"
    std::shared_ptr<const LidarFrame> lidar;
    std::shared_ptr<const ImuSample>  imu;
    std::shared_ptr<const CameraFrame> camera;
};

// ─── Device information ────────────────────────────────────────────────────

struct DeviceInfo {
    std::string serial_number;
    std::string firmware_version;
    std::string model_name;
};

// ─── Status / error codes ──────────────────────────────────────────────────

enum class Status : int {
    OK                  =  0,
    NotConnected        = -1,
    Timeout             = -2,
    TransportError      = -3,
    InvalidParameter    = -4,
    NotSupported        = -5,
    AlreadyStreaming    = -6,
    InternalError       = -99,
};

inline const char* status_string(Status s) {
    switch (s) {
        case Status::OK:               return "OK";
        case Status::NotConnected:     return "NotConnected";
        case Status::Timeout:          return "Timeout";
        case Status::TransportError:   return "TransportError";
        case Status::InvalidParameter: return "InvalidParameter";
        case Status::NotSupported:     return "NotSupported";
        case Status::AlreadyStreaming: return "AlreadyStreaming";
        case Status::InternalError:    return "InternalError";
    }
    return "Unknown";
}

// ─── Callback typedefs ─────────────────────────────────────────────────────

using LidarCallback  = std::function<void(std::shared_ptr<const LidarFrame>)>;
using ImuCallback    = std::function<void(std::shared_ptr<const ImuSample>)>;
using CameraCallback = std::function<void(std::shared_ptr<const CameraFrame>)>;
using SyncCallback   = std::function<void(std::shared_ptr<const SyncBundle>)>;

} // namespace thunderbird
