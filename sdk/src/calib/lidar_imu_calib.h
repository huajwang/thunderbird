// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — BALM LiDAR-IMU Extrinsic Calibration
// ─────────────────────────────────────────────────────────────────────────────
//
// Voxel-based multi-scale calibration of the LiDAR-to-IMU transform.
// Inspired by the BALM algorithm from OpenCalib (Apache 2.0, PJLab-ADG)
// but reimplemented using Thunderbird types with Ceres as optional backend.
//
// Algorithm overview:
//   1. Accumulate deskewed LiDAR scans with associated IMU poses
//   2. Build voxel grid and extract surface/corner features via eigenvalues
//   3. Optimize the 6-DOF LiDAR→IMU transform to minimize feature scatter
//   4. Multi-round coarse-to-fine refinement
//
// Requires: Ceres Solver (guarded by THUNDERBIRD_HAS_CERES)
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/types.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace thunderbird::calib {

// ─── Input data types ───────────────────────────────────────────────────────

/// A 3D point with coordinates in the LiDAR frame.
struct LidarPoint3D {
    double x, y, z;
    double intensity;
    int    ring;        ///< Laser ring index (for feature extraction).
};

/// A single LiDAR scan with its associated IMU-derived pose.
struct CalibFrame {
    std::vector<LidarPoint3D> points;

    /// IMU-derived pose at scan time (world ← IMU).
    /// Rotation as quaternion [w,x,y,z], translation [x,y,z].
    double pose_rotation[4] = {1, 0, 0, 0};
    double pose_translation[3] = {0, 0, 0};
};

// ─── Configuration ──────────────────────────────────────────────────────────

struct LidarImuCalibConfig {
    /// Number of optimization rounds.
    int num_rounds = 20;

    /// Initial voxel size (meters).
    double voxel_size = 1.0;

    /// Maximum octree subdivision depth.
    int max_octree_depth = 5;

    /// Minimum points in a voxel for feature extraction.
    int min_points_per_voxel = 20;

    /// Eigenvalue ratio threshold for surface features (higher = stricter).
    double surface_eigen_ratio = 16.0;

    /// Eigenvalue ratio threshold for corner features.
    double corner_eigen_ratio = 9.0;

    /// Ceres solver max iterations per round.
    int solver_max_iterations = 30;

    /// Downsampling voxel grid size (meters, 0 = disabled).
    double downsample_size = 0.2;
};

// ─── Result ─────────────────────────────────────────────────────────────────

struct LidarImuCalibResult {
    SensorExtrinsic imu_T_lidar;   ///< Optimized LiDAR→IMU transform.
    double final_cost = 0.0;       ///< Final Ceres cost.
    int    num_residuals = 0;      ///< Number of residual terms.
    int    rounds_completed = 0;   ///< Optimization rounds completed.
    bool   converged = false;      ///< Whether Ceres converged.
};

// ─── Public API ─────────────────────────────────────────────────────────────

/// Check whether BALM calibration is available (compiled with Ceres support).
bool isLidarImuCalibAvailable();

/// Calibrate LiDAR-IMU extrinsic using BALM voxel optimization.
///
/// @param frames     Vector of LiDAR scans with associated IMU poses.
/// @param initial    Initial estimate of imu_T_lidar.
/// @param config     Optimizer configuration.
/// @return Optimized calibration result.
LidarImuCalibResult calibrateLidarImu(
    const std::vector<CalibFrame>& frames,
    const SensorExtrinsic& initial,
    const LidarImuCalibConfig& config = {});

using LidarImuProgressCallback = std::function<void(int round, double cost)>;

/// Calibrate with progress callback.
LidarImuCalibResult calibrateLidarImuWithProgress(
    const std::vector<CalibFrame>& frames,
    const SensorExtrinsic& initial,
    const LidarImuCalibConfig& config,
    LidarImuProgressCallback progress);

}  // namespace thunderbird::calib
