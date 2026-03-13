// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Calibration Bundle
// ─────────────────────────────────────────────────────────────────────────────
//
// Aggregates all calibration parameters for a multi-sensor system:
//
//   • Camera intrinsics (per camera)
//   • Sensor extrinsics (LiDAR↔IMU, Camera↔IMU, Camera↔LiDAR)
//   • IMU noise model
//   • Camera-IMU temporal offsets (from Kalibr)
//
// The bundle can be serialized to/from YAML for persistent storage.
// YAML files produced by Kalibr can be imported via the companion
// kalibr_import module (Phase 2).
//
// Design:
//   • Up to kMaxCameras cameras (compile-time constant, default 4).
//   • All transforms use the SensorExtrinsic type from types.h.
//   • IMU noise uses the same spectral density convention as
//     Kalibr's imu.yaml. The SLAM engine uses this type directly
//     via CalibrationBundle.
//   • Thread safety: immutable after load; copy for mutation.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/types.h"

#include <cstdint>
#include <string>
#include <vector>

#include "thunderbird/export.h"
#include "thunderbird/abi.h"

namespace thunderbird {
THUNDERBIRD_ABI_NAMESPACE_BEGIN

// ─── IMU noise model ──────────────────────────────────────────────────────────────

/// Continuous-time spectral densities for IMU error modelling.
/// Used by SLAM (ESIKF Q-matrix) and calibration tools (Kalibr).
struct ImuNoiseParams {
    double gyro_noise    = 1.0e-3;   ///< rad/s/√Hz   gyroscope white noise
    double accel_noise   = 1.0e-2;   ///< m/s²/√Hz    accelerometer white noise
    double gyro_bias_rw  = 1.0e-5;   ///< rad/s²/√Hz  gyroscope bias random walk
    double accel_bias_rw = 1.0e-4;   ///< m/s³/√Hz    accelerometer bias random walk
};

// ─── Per-camera calibration ─────────────────────────────────────────────────

/// All calibration data for a single camera in the rig.
struct CameraCalibration {
    std::string         label;         ///< e.g. "cam0", "front_left"
    CameraIntrinsics    intrinsics;    ///< focal length, principal point, distortion

    /// Transform from this camera's frame to the IMU frame.
    /// p_imu = imu_T_camera * p_camera
    SensorExtrinsic     imu_T_camera;

    /// Temporal offset: camera_timestamp = imu_timestamp + time_offset_ns.
    /// Estimated by Kalibr; 0 if not calibrated.
    int64_t             time_offset_ns{0};
};

// ─── Calibration bundle ─────────────────────────────────────────────────────

/// Maximum number of cameras in a calibration bundle.
static constexpr size_t kMaxCameras = 4;

/// Complete calibration state for a multi-sensor rig.
///
/// Aggregates camera intrinsics/extrinsics, LiDAR-IMU extrinsic, and IMU
/// noise parameters into a single structure that can be loaded from YAML
/// and passed to SLAM, perception, and recording subsystems.
struct THUNDERBIRD_API CalibrationBundle {
    // ── LiDAR ↔ IMU ────────────────────────────────────────────────────
    /// p_imu = imu_T_lidar * p_lidar
    SensorExtrinsic     imu_T_lidar;

    /// If true, SLAM may refine imu_T_lidar online.
    bool                refine_imu_T_lidar{false};

    // ── Cameras ─────────────────────────────────────────────────────────
    std::vector<CameraCalibration> cameras;

    // ── IMU noise ───────────────────────────────────────────────────────
    ImuNoiseParams      imu_noise;

    // ── Derived transforms (computed from above) ────────────────────────

    /// Compute lidar_T_camera for camera index `i`.
    /// lidar_T_camera = imu_T_lidar.inverse() ∘ imu_T_camera
    [[nodiscard]] SensorExtrinsic lidar_T_camera(size_t i) const {
        if (i >= cameras.size()) return {};
        return imu_T_lidar.inverse().compose(cameras[i].imu_T_camera);
    }

    /// Compute camera_T_lidar for camera index `i`.
    [[nodiscard]] SensorExtrinsic camera_T_lidar(size_t i) const {
        return lidar_T_camera(i).inverse();
    }

    // ── Persistence ─────────────────────────────────────────────────────

    /// Load calibration from a YAML file.
    ///
    /// Expected top-level keys:
    ///   imu_T_lidar:       { rotation: [w,x,y,z], translation: [x,y,z] }
    ///   refine_imu_T_lidar: bool
    ///   imu_noise:         { gyro_noise, accel_noise, gyro_bias_rw, accel_bias_rw }
    ///   cameras:
    ///     - label: "cam0"
    ///       intrinsics: { fx, fy, cx, cy, width, height, distortion_model, distortion_coeffs }
    ///       imu_T_camera: { rotation, translation }
    ///       time_offset_ns: int
    ///
    /// @param path  File path to the YAML file.
    /// @return true if loaded successfully; false on parse error or missing file.
    bool load_yaml(const std::string& path);

    /// Save calibration to a YAML file.
    ///
    /// @param path  Output file path.
    /// @return true if saved successfully; false on I/O error.
    bool save_yaml(const std::string& path) const;
};

THUNDERBIRD_ABI_NAMESPACE_END
} // namespace thunderbird
