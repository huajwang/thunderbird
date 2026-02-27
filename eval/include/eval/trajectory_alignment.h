// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Evaluation Framework: Trajectory Alignment
// ─────────────────────────────────────────────────────────────────────────────
//
// Implements closed-form SE(3) and Sim(3) trajectory alignment using
// Horn's unit-quaternion method (1987) with optional scale recovery and
// iterative outlier rejection.
//
// References:
//   [1] Horn, "Closed-form Solution of Absolute Orientation using Unit
//       Quaternions", JOSA-A 1987.
//   [2] Umeyama, "Least-Squares Estimation of Transformation Parameters
//       Between Two Point Patterns", TPAMI 1991.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace eval {

// ═════════════════════════════════════════════════════════════════════════════
//  AlignmentResult — output of trajectory alignment
// ═════════════════════════════════════════════════════════════════════════════

struct AlignmentResult {
    // ── Recovered transformation: T_gt_from_est ─────────────────────────
    std::array<double,4> rotation{1,0,0,0};  ///< [w,x,y,z] quaternion
    std::array<double,3> translation{};       ///< metres
    double               scale{1.0};          ///< 1.0 for SE(3), free for Sim(3)

    // ── Diagnostics ─────────────────────────────────────────────────────
    size_t inlier_count{0};     ///< number of inlier pairs used
    size_t outlier_count{0};    ///< number of pairs rejected as outliers
    size_t total_pairs{0};      ///< input pair count
    size_t iterations{0};       ///< outlier rejection iterations performed
    double rmse_before{0.0};    ///< ATE RMSE prior to alignment (metres)
    double rmse_after{0.0};     ///< ATE RMSE after alignment (metres)
    bool   converged{false};    ///< did outlier rejection converge?
};

// ═════════════════════════════════════════════════════════════════════════════
//  AlignmentConfig
// ═════════════════════════════════════════════════════════════════════════════

struct AlignmentConfig {
    bool   estimate_scale{false};       ///< false = SE(3), true = Sim(3)
    bool   outlier_rejection{false};    ///< enable iterative outlier removal
    double outlier_threshold_sigma{3.0};///< reject pairs > N*σ from mean error
    size_t max_outlier_iters{10};       ///< max outlier rejection iterations
    double min_inlier_ratio{0.5};       ///< stop if inliers drop below this
};

// ═════════════════════════════════════════════════════════════════════════════
//  Point pair for alignment
// ═════════════════════════════════════════════════════════════════════════════

struct PointPair {
    std::array<double,3> source;  ///< estimated position
    std::array<double,3> target;  ///< ground truth position
};

// ═════════════════════════════════════════════════════════════════════════════
//  Core alignment functions
// ═════════════════════════════════════════════════════════════════════════════

/// Compute SE(3) or Sim(3) alignment using Horn's quaternion method.
///
/// Finds (s, R, t) such that:   target ≈ s * R * source + t
///
/// If cfg.estimate_scale is false, s is fixed at 1.0 (pure SE(3)).
///
/// @param pairs   Matched point pairs (source=estimated, target=GT).
/// @param cfg     Alignment configuration.
/// @return        AlignmentResult with transformation and diagnostics.
AlignmentResult alignTrajectories(const std::vector<PointPair>& pairs,
                                   AlignmentConfig cfg = {});

/// Apply the recovered alignment to a set of source points.
/// result[i] = scale * R * source[i] + t
std::vector<std::array<double,3>> applyAlignment(
    const std::vector<std::array<double,3>>& source,
    const AlignmentResult& alignment);

/// Apply alignment to a single point.
std::array<double,3> applyAlignment(const std::array<double,3>& point,
                                     const AlignmentResult& alignment);

// ═════════════════════════════════════════════════════════════════════════════
//  Quaternion/rotation utilities (exposed for testing)
// ═════════════════════════════════════════════════════════════════════════════

namespace horn_detail {

/// Build Horn's 4×4 symmetric matrix N from the cross-covariance.
/// Returns the matrix in row-major order (16 doubles).
std::array<double,16> buildN(const std::vector<PointPair>& pairs,
                              const std::array<double,3>& centroid_src,
                              const std::array<double,3>& centroid_tgt);

/// Find the eigenvector of a 4×4 symmetric matrix corresponding to
/// the largest eigenvalue.  Uses Jacobi iteration (exact for 4×4).
/// Returns [w,x,y,z] unit quaternion.
std::array<double,4> maxEigenvector4x4(const std::array<double,16>& N);

/// Rotate vector v by unit quaternion q: q⊗[0,v]⊗q*.
std::array<double,3> rotateByQuat(const std::array<double,4>& q,
                                   const std::array<double,3>& v);

/// Compute RMSE from aligned errors.
double computeRmse(const std::vector<PointPair>& pairs,
                   const std::array<double,4>& quat,
                   const std::array<double,3>& trans,
                   double scale);

} // namespace horn_detail

} // namespace eval
