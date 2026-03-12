// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Rigid Transform Solver (Horn's SVD Method)
// ─────────────────────────────────────────────────────────────────────────────
//
// Clean-room implementation of the optimal rigid body transformation from
// corresponding 3D point pairs, based on:
//
//   B.K.P. Horn, "Closed-form solution of absolute orientation using
//   unit quaternions," Journal of the Optical Society of America A, 1987.
//
// The implementation uses a self-contained 3×3 SVD (Jacobi method) to
// avoid any external dependency.
//
// This algorithm is public domain math (textbook algorithm) — not derived
// from any GPL-licensed source code.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/types.h"

#include <cstddef>
#include <vector>

namespace thunderbird::calib {

/// Result of a rigid transform estimation.
struct RigidTransformResult {
    SensorExtrinsic transform;  ///< The estimated rigid transform (R, t).
    double rmse = 0.0;          ///< Root mean square error of residuals.
    int    num_inliers = 0;     ///< Number of point pairs used.
    bool   valid = false;       ///< Whether the estimation succeeded.
};

/// Estimate the rigid transform (R, t) from corresponding 3D point pairs.
///
/// Given source points {p_i} and target points {q_i}, finds the
/// rotation R and translation t that minimizes:
///
///   sum_i || q_i - (R * p_i + t) ||²
///
/// The result transform maps source → target:
///   q = transform.rotation * p + transform.translation
///
/// @param source     Source points, flat array [x0,y0,z0, x1,y1,z1, ...].
/// @param target     Target points, same layout.
/// @param n_points   Number of point pairs (minimum 3).
/// @return RigidTransformResult with the estimated transform and RMSE.
RigidTransformResult solveRigidTransform(const double* source,
                                         const double* target,
                                         int n_points);

/// Same as above but with std::vector interface.
RigidTransformResult solveRigidTransform(const std::vector<double>& source,
                                         const std::vector<double>& target);

/// RANSAC wrapper: estimate rigid transform robustly with outlier rejection.
///
/// @param source        Source points, flat [x,y,z,...].
/// @param target        Target points.
/// @param n_points      Total number of point pairs.
/// @param threshold     Inlier distance threshold (meters).
/// @param max_iter      Maximum RANSAC iterations.
/// @param inlier_ratio  Minimum inlier ratio to accept result.
/// @return Best transform with the most inliers.
RigidTransformResult solveRigidTransformRANSAC(
    const double* source, const double* target, int n_points,
    double threshold = 0.05, int max_iter = 200, double inlier_ratio = 0.5);

}  // namespace thunderbird::calib
