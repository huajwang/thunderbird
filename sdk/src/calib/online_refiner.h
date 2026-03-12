// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Online Extrinsic Refinement
// ─────────────────────────────────────────────────────────────────────────────
//
// Per-frame ground plane extraction to continuously refine the LiDAR→IMU
// extrinsic (pitch, roll, and height). Inspired by OpenCalib's online_calib
// module (Apache 2.0, PJLab-ADG).
//
// Algorithm:
//   1. For each incoming LiDAR scan, extract ground points (low-Z region)
//   2. Fit a plane via PCA (covariance eigendecomposition)
//   3. Compute the rotation aligning ground normal to [0,0,1]
//   4. Accumulate corrections via confidence-weighted quaternion SLERP
//   5. Emit pitch/roll/height corrections to the SLAM engine
//
// This module is dependency-free — no PCL, Eigen, or OpenCV required.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/types.h"

#include <cstddef>
#include <cstdint>
#include <functional>

namespace thunderbird::calib {

/// Configuration for the online extrinsic refiner.
struct OnlineRefinerConfig {
    /// Maximum LiDAR point height (in LiDAR frame) for ground candidate.
    double ground_max_height = -0.5;

    /// Minimum number of ground points for valid plane fit.
    int min_ground_points = 100;

    /// Minimum ground normal Z component (rejects steep planes).
    double min_normal_z = 0.9;

    /// Minimum height (ground intercept) to accept a correction.
    double min_height = 1.0;

    /// Maximum correction magnitude before warning (degrees).
    double max_correction_deg = 2.0;

    /// Maximum height correction before warning (meters).
    double max_correction_height = 0.10;
};

/// Current extrinsic correction state.
struct ExtrinsicCorrection {
    /// Correction rotation as quaternion [w,x,y,z].
    /// Represents the accumulated ground-to-LiDAR rotation.
    double rotation[4] = {1, 0, 0, 0};

    /// Height of LiDAR above ground (meters).
    double height = 0.0;

    /// Accumulated confidence (higher = more frames processed).
    double confidence = 0.0;

    /// Number of frames processed.
    int frame_count = 0;

    /// Whether the correction exceeds safety thresholds.
    bool warning = false;
};

/// Point input for the refiner (flat struct, no inheritance).
struct RefinerPoint {
    double x, y, z;
};

/// Online extrinsic refiner.
///
/// Feed LiDAR scans via `processFrame()` and read corrections from
/// `correction()`. Thread-safe for single-writer / single-reader.
class OnlineRefiner {
public:
    explicit OnlineRefiner(const OnlineRefinerConfig& config = {});
    ~OnlineRefiner();

    // Non-copyable
    OnlineRefiner(const OnlineRefiner&) = delete;
    OnlineRefiner& operator=(const OnlineRefiner&) = delete;

    /// Process one LiDAR scan.
    /// @param points     Flat array of 3D points [x,y,z,...] in LiDAR frame.
    /// @param n_points   Number of points.
    /// @return true if a ground plane was successfully extracted.
    bool processFrame(const RefinerPoint* points, int n_points);

    /// Get the current accumulated correction.
    ExtrinsicCorrection correction() const;

    /// Reset the accumulator.
    void reset();

private:
    OnlineRefinerConfig config_;
    ExtrinsicCorrection correction_;
};

}  // namespace thunderbird::calib
