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

#include "thunderbird/export.h"
#include "thunderbird/abi.h"

namespace thunderbird {
THUNDERBIRD_ABI_NAMESPACE_BEGIN

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

// ─── Camera intrinsic calibration ──────────────────────────────────────────

/// Distortion model identifier (matches Kalibr / OpenCV conventions).
enum class DistortionModel : uint8_t {
    None              = 0,   ///< No distortion
    RadialTangential  = 1,   ///< OpenCV radtan: k1, k2, p1, p2 [, k3]
    Equidistant       = 2,   ///< Fisheye: k1, k2, k3, k4
    FieldOfView       = 3,   ///< FOV model: single ω parameter
};

inline const char* distortion_model_name(DistortionModel m) {
    switch (m) {
        case DistortionModel::None:             return "none";
        case DistortionModel::RadialTangential: return "radtan";
        case DistortionModel::Equidistant:      return "equidistant";
        case DistortionModel::FieldOfView:      return "fov";
    }
    return "unknown";
}

/// Camera intrinsic parameters (pinhole model + distortion).
///
/// Projection: pixel = [fx*X/Z + cx, fy*Y/Z + cy] after distortion.
/// Compatible with Kalibr camchain.yaml and OpenCV camera matrices.
struct CameraIntrinsics {
    double fx{0};               ///< focal length x (pixels)
    double fy{0};               ///< focal length y (pixels)
    double cx{0};               ///< principal point x (pixels)
    double cy{0};               ///< principal point y (pixels)

    uint32_t width{0};          ///< image width (pixels)
    uint32_t height{0};         ///< image height (pixels)

    DistortionModel distortion_model{DistortionModel::None};

    /// Distortion coefficients. Interpretation depends on distortion_model:
    ///   RadialTangential: [k1, k2, p1, p2, k3, 0, 0, 0]
    ///   Equidistant:      [k1, k2, k3, k4, 0, 0, 0, 0]
    ///   FieldOfView:      [ω,  0,  0,  0,  0, 0, 0, 0]
    ///   None:             unused
    std::array<double, 8> distortion_coeffs{};

    /// True if parameters have been set (not default-constructed).
    [[nodiscard]] bool valid() const noexcept {
        return fx > 0 && fy > 0 && width > 0 && height > 0;
    }
};

// ─── Rigid-body transform (sensor extrinsic) ──────────────────────────────

/// A rigid-body SE(3) transform between two sensor frames.
///
/// Convention: transforms a point from the *source* frame to the *target*
/// frame.  For example, `imu_T_lidar` transforms a point from LiDAR frame
/// to IMU frame: p_imu = R * p_lidar + t.
///
/// Quaternion: Hamilton convention [w, x, y, z], unit norm.
struct SensorExtrinsic {
    std::array<double, 4> rotation{1.0, 0.0, 0.0, 0.0};   ///< [w,x,y,z]
    std::array<double, 3> translation{0.0, 0.0, 0.0};      ///< metres

    /// Check if this is the identity transform.
    [[nodiscard]] bool is_identity() const noexcept {
        return rotation[0] == 1.0 && rotation[1] == 0.0 &&
               rotation[2] == 0.0 && rotation[3] == 0.0 &&
               translation[0] == 0.0 && translation[1] == 0.0 &&
               translation[2] == 0.0;
    }

    /// Compute the inverse transform.
    /// If this is A_T_B (B→A), inverse() returns B_T_A (A→B).
    [[nodiscard]] SensorExtrinsic inverse() const noexcept {
        // q_inv = conjugate (since unit quaternion)
        const double w = rotation[0], x = -rotation[1],
                     y = -rotation[2], z = -rotation[3];

        // R_inv * (-t) = q_inv ⊗ (-t) ⊗ q
        // For translation: t_inv = -R^T * t
        // Using quaternion rotation of the negated translation:
        const double tx = -translation[0], ty = -translation[1],
                     tz = -translation[2];

        // Rotate (tx, ty, tz) by quaternion (w, x, y, z):
        // v' = q * v * q^-1, but since q IS the inverse, we use it directly
        const double t2_0 = w*w + x*x - y*y - z*z;
        const double t2_1 = 2.0*(x*y - w*z);
        const double t2_2 = 2.0*(x*z + w*y);
        const double t2_3 = 2.0*(x*y + w*z);
        const double t2_4 = w*w - x*x + y*y - z*z;
        const double t2_5 = 2.0*(y*z - w*x);
        const double t2_6 = 2.0*(x*z - w*y);
        const double t2_7 = 2.0*(y*z + w*x);
        const double t2_8 = w*w - x*x - y*y + z*z;

        return {{w, x, y, z},  // conjugate quaternion IS the inverse rotation
                {t2_0*tx + t2_1*ty + t2_2*tz,
                 t2_3*tx + t2_4*ty + t2_5*tz,
                 t2_6*tx + t2_7*ty + t2_8*tz}};
    }

    /// Compose two transforms: result = this ∘ other.
    /// If this is A_T_B and other is B_T_C, result is A_T_C.
    [[nodiscard]] SensorExtrinsic compose(const SensorExtrinsic& other) const noexcept {
        // q_result = q_this ⊗ q_other  (Hamilton product)
        const double aw = rotation[0], ax = rotation[1],
                     ay = rotation[2], az = rotation[3];
        const double bw = other.rotation[0], bx = other.rotation[1],
                     by = other.rotation[2], bz = other.rotation[3];

        const double rw = aw*bw - ax*bx - ay*by - az*bz;
        const double rx = aw*bx + ax*bw + ay*bz - az*by;
        const double ry = aw*by - ax*bz + ay*bw + az*bx;
        const double rz = aw*bz + ax*by - ay*bx + az*bw;

        // t_result = R_this * t_other + t_this
        // Rotate other.translation by this quaternion:
        const double ox = other.translation[0], oy = other.translation[1],
                     oz = other.translation[2];

        const double r00 = 1 - 2*(ay*ay + az*az);
        const double r01 = 2*(ax*ay - aw*az);
        const double r02 = 2*(ax*az + aw*ay);
        const double r10 = 2*(ax*ay + aw*az);
        const double r11 = 1 - 2*(ax*ax + az*az);
        const double r12 = 2*(ay*az - aw*ax);
        const double r20 = 2*(ax*az - aw*ay);
        const double r21 = 2*(ay*az + aw*ax);
        const double r22 = 1 - 2*(ax*ax + ay*ay);

        return {{rw, rx, ry, rz},
                {r00*ox + r01*oy + r02*oz + translation[0],
                 r10*ox + r11*oy + r12*oz + translation[1],
                 r20*ox + r21*oy + r22*oz + translation[2]}};
    }
};

// ─── Callback typedefs ─────────────────────────────────────────────────────

using LidarCallback  = std::function<void(std::shared_ptr<const LidarFrame>)>;
using ImuCallback    = std::function<void(std::shared_ptr<const ImuSample>)>;
using CameraCallback = std::function<void(std::shared_ptr<const CameraFrame>)>;
using SyncCallback   = std::function<void(std::shared_ptr<const SyncBundle>)>;

THUNDERBIRD_ABI_NAMESPACE_END
} // namespace thunderbird
