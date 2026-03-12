// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Kalibr Result Import
// ─────────────────────────────────────────────────────────────────────────────
//
// Parses Kalibr calibration output files (camchain, IMU) and converts them
// to a Thunderbird CalibrationBundle.
//
// Kalibr output files:
//   - camchain-imucam-*.yaml: Camera intrinsics, extrinsics, time offsets
//   - imu-*.yaml:             IMU noise parameters (spectral densities)
//
// The importer handles:
//   - Kalibr's 4×4 T_cam_imu → Thunderbird's imu_T_camera (inversion)
//   - Kalibr's distortion model names → DistortionModel enum
//   - Kalibr's timeshift_cam_imu (seconds) → time_offset_ns (nanoseconds)
//   - IMU spectral densities (same convention, direct copy)
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/calibration.h"

#include <string>

namespace thunderbird::calib {

/// Import results from a Kalibr camera chain YAML (camchain-imucam-*.yaml).
///
/// Populates bundle.cameras with intrinsics, extrinsics and time offsets.
/// Does NOT modify imu_T_lidar or imu_noise (those come from separate sources).
///
/// @param path    Path to the camchain YAML file.
/// @param bundle  Output calibration bundle (cameras vector is cleared first).
/// @return true on success; false on parse error or missing file.
bool importKalibrCamchain(const std::string& path, CalibrationBundle& bundle);

/// Import IMU noise parameters from a Kalibr IMU YAML (imu-*.yaml).
///
/// Populates bundle.imu_noise with spectral density values.
///
/// @param path    Path to the IMU YAML file.
/// @param bundle  Output calibration bundle (only imu_noise is modified).
/// @return true on success; false on parse error or missing file.
bool importKalibrImu(const std::string& path, CalibrationBundle& bundle);

/// Import both camchain and IMU YAML files in one call.
///
/// @param camchain_path  Path to camchain YAML.
/// @param imu_path       Path to IMU YAML.
/// @param bundle         Output calibration bundle.
/// @return true if both imports succeed.
bool importKalibrFull(const std::string& camchain_path,
                      const std::string& imu_path,
                      CalibrationBundle& bundle);

}  // namespace thunderbird::calib
