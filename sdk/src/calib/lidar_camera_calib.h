// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — LiDAR-Camera Extrinsic Auto-Calibration
// ─────────────────────────────────────────────────────────────────────────────
//
// Multi-stage random search optimizer for LiDAR-camera extrinsic calibration.
// Inspired by OpenCalib/SensorsCalibration (Apache 2.0, PJLab-ADG) but
// reimplemented using Thunderbird types and dependency-free math.
//
// Algorithm:
//   1. Project LiDAR points onto the camera image using initial extrinsic
//   2. Count how many projections land on edge pixels (high gradient)
//   3. Generate random 6-DOF perturbations, keep the best
//   4. Repeat across multiple stages with decreasing search ranges
//
// No OpenCV/PCL/Eigen dependency — uses Thunderbird types only.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/types.h"

#include <cstdint>
#include <functional>
#include <vector>

namespace thunderbird::calib {

/// Configuration for the LiDAR-camera calibration optimizer.
struct LidarCameraCalibConfig {
    /// Number of random search iterations per stage.
    int samples_per_stage = 5000;

    /// Number of refinement stages (each stage narrows the search range).
    int num_stages = 4;

    /// Initial rotation search range (degrees).
    double init_rot_range_deg = 1.0;

    /// Initial translation search range (meters).
    double init_trans_range_m = 0.05;

    /// Range reduction factor between stages.
    double range_decay = 0.5;

    /// Edge detection threshold (pixel gradient magnitude, 0-255).
    double edge_threshold = 30.0;

    /// Minimum LiDAR point depth for projection (meters).
    double min_depth = 0.5;
};

/// Result of LiDAR-camera calibration.
struct LidarCameraCalibResult {
    SensorExtrinsic camera_T_lidar;  ///< Optimized transform.
    double score = 0.0;              ///< Final edge alignment score [0, 1].
    int total_projected = 0;         ///< Points successfully projected.
    int edge_matches = 0;            ///< Points matching edges.
    bool converged = false;          ///< Whether optimization converged.
};

/// Grayscale image wrapper for edge-based cost function.
/// The caller provides a flat grayscale image buffer.
struct GrayImage {
    const uint8_t* data = nullptr;   ///< Row-major grayscale pixels.
    int width = 0;
    int height = 0;
};

/// A 3D point with intensity (flat struct, no inheritance).
struct CalibPoint3D {
    double x, y, z;
    double intensity;
};

/// Run LiDAR-camera extrinsic calibration.
///
/// @param points       LiDAR point cloud (in LiDAR frame).
/// @param n_points     Number of points.
/// @param edge_image   Grayscale edge/gradient image from the camera.
/// @param intrinsics   Camera intrinsic parameters.
/// @param initial      Initial estimate of camera_T_lidar.
/// @param config       Optimizer configuration.
/// @return Optimized calibration result.
LidarCameraCalibResult calibrateLidarCamera(
    const CalibPoint3D* points, int n_points,
    const GrayImage& edge_image,
    const CameraIntrinsics& intrinsics,
    const SensorExtrinsic& initial,
    const LidarCameraCalibConfig& config = {});

/// Optional progress callback: (stage, samples_done, current_score).
using CalibProgressCallback = std::function<void(int, int, double)>;

/// Run with progress callback.
LidarCameraCalibResult calibrateLidarCameraWithProgress(
    const CalibPoint3D* points, int n_points,
    const GrayImage& edge_image,
    const CameraIntrinsics& intrinsics,
    const SensorExtrinsic& initial,
    const LidarCameraCalibConfig& config,
    CalibProgressCallback progress);

}  // namespace thunderbird::calib
