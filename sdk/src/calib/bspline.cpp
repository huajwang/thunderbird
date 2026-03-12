// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Uniform Cubic B-Spline Implementation
// ─────────────────────────────────────────────────────────────────────────────
//
// Uniform cubic B-spline evaluation using the standard basis matrix.
//
// The cubic B-spline basis matrix M (for uniform knots) is:
//
//          1  [ -1   3  -3   1 ]
//   M = ─── × [  3  -6   3   0 ]
//          6  [ -3   0   3   0 ]
//             [  1   4   1   0 ]
//
// For parameter u ∈ [0,1) in segment i, the spline value is:
//
//   S(u) = [u³ u² u 1] × M × [P_{i-1}  P_i  P_{i+1}  P_{i+2}]^T
//
// Derivatives are obtained by differentiating the power vector:
//   S'(u)  = [3u² 2u 1 0] × M × P  / dt
//   S''(u) = [6u  2  0 0] × M × P  / dt²
//
// ─────────────────────────────────────────────────────────────────────────────
#include "bspline.h"

#include <cmath>
#include <cstring>

namespace thunderbird::calib {

UniformCubicBSpline::UniformCubicBSpline(int dim)
    : dim_(dim) {
    assert(dim > 0);
}

bool UniformCubicBSpline::initFromData(double t0, double t1,
                                       const double* data, int n_samples) {
    if (!data || n_samples < 4 || t1 <= t0 || dim_ <= 0) return false;

    // Use the data samples directly as control points.
    // For a uniform spline with the same number of control points as samples,
    // the spline approximately interpolates the data (with slight smoothing
    // inherent to B-splines, which is desirable for noisy IMU data).
    n_ctrl_ = n_samples;
    t0_ = t0;
    dt_ = (t1 - t0) / static_cast<double>(n_samples - 1);

    coeffs_.resize(static_cast<size_t>(n_ctrl_) * static_cast<size_t>(dim_));
    std::memcpy(coeffs_.data(), data,
                static_cast<size_t>(n_ctrl_) * static_cast<size_t>(dim_) * sizeof(double));
    return true;
}

bool UniformCubicBSpline::initFromCoefficients(double t0, double dt,
                                               const double* coeffs, int n_ctrl) {
    if (!coeffs || n_ctrl < 4 || dt <= 0.0 || dim_ <= 0) return false;

    n_ctrl_ = n_ctrl;
    t0_ = t0;
    dt_ = dt;

    coeffs_.resize(static_cast<size_t>(n_ctrl_) * static_cast<size_t>(dim_));
    std::memcpy(coeffs_.data(), coeffs,
                static_cast<size_t>(n_ctrl_) * static_cast<size_t>(dim_) * sizeof(double));
    return true;
}

bool UniformCubicBSpline::segmentParam(double t, int& seg, double& u) const {
    if (n_ctrl_ < 4) return false;

    // Valid interval: [t0_ + dt_, t0_ + (n_ctrl_ - 2) * dt_]
    double t_min = t0_ + dt_;
    double t_max = t0_ + static_cast<double>(n_ctrl_ - 2) * dt_;

    if (t < t_min || t > t_max) return false;

    // Segment index: which group of 4 control points
    double s = (t - t0_) / dt_;
    seg = static_cast<int>(std::floor(s)) - 1;  // -1 because cubic needs P_{i-1}..P_{i+2}

    // Clamp to valid range
    if (seg < 0) seg = 0;
    if (seg > n_ctrl_ - 4) seg = n_ctrl_ - 4;

    // Local parameter within segment
    u = s - static_cast<double>(seg + 1);

    // Clamp u for numerical safety
    if (u < 0.0) u = 0.0;
    if (u > 1.0) u = 1.0;

    return true;
}

// Evaluate basis functions: [u³ u² u 1] × M
// Returns 4 weights for the 4 control points of this segment.
static void basisWeights(double u, double w[4]) {
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double inv6 = 1.0 / 6.0;

    w[0] = inv6 * (-u3 + 3.0 * u2 - 3.0 * u + 1.0);
    w[1] = inv6 * ( 3.0 * u3 - 6.0 * u2 + 4.0);
    w[2] = inv6 * (-3.0 * u3 + 3.0 * u2 + 3.0 * u + 1.0);
    w[3] = inv6 * u3;
}

// First derivative basis: [3u² 2u 1 0] × M / dt
static void basisWeightsD1(double u, double w[4]) {
    const double u2 = u * u;
    const double inv6 = 1.0 / 6.0;

    w[0] = inv6 * (-3.0 * u2 + 6.0 * u - 3.0);
    w[1] = inv6 * ( 9.0 * u2 - 12.0 * u);
    w[2] = inv6 * (-9.0 * u2 + 6.0 * u + 3.0);
    w[3] = inv6 * ( 3.0 * u2);
}

// Second derivative basis: [6u 2 0 0] × M / dt²
static void basisWeightsD2(double u, double w[4]) {
    const double inv6 = 1.0 / 6.0;

    w[0] = inv6 * (-6.0 * u + 6.0);
    w[1] = inv6 * ( 18.0 * u - 12.0);
    w[2] = inv6 * (-18.0 * u + 6.0);
    w[3] = inv6 * ( 6.0 * u);
}

bool UniformCubicBSpline::eval(double t, double* result) const {
    int seg;
    double u;
    if (!segmentParam(t, seg, u)) return false;

    double w[4];
    basisWeights(u, w);

    // Weighted sum of 4 control points
    for (int j = 0; j < dim_; ++j) {
        result[j] = 0.0;
        for (int k = 0; k < 4; ++k) {
            result[j] += w[k] * coeffs_[static_cast<size_t>(seg + k) * static_cast<size_t>(dim_) + static_cast<size_t>(j)];
        }
    }
    return true;
}

bool UniformCubicBSpline::evalDerivative(double t, int order, double* result) const {
    if (order == 0) return eval(t, result);

    int seg;
    double u;
    if (!segmentParam(t, seg, u)) return false;

    double w[4];
    double scale = 1.0;

    if (order == 1) {
        basisWeightsD1(u, w);
        scale = 1.0 / dt_;
    } else if (order == 2) {
        basisWeightsD2(u, w);
        scale = 1.0 / (dt_ * dt_);
    } else {
        // Higher order derivatives of a cubic are zero
        for (int j = 0; j < dim_; ++j) result[j] = 0.0;
        return true;
    }

    for (int j = 0; j < dim_; ++j) {
        result[j] = 0.0;
        for (int k = 0; k < 4; ++k) {
            result[j] += w[k] * coeffs_[static_cast<size_t>(seg + k) * static_cast<size_t>(dim_) + static_cast<size_t>(j)];
        }
        result[j] *= scale;
    }
    return true;
}

}  // namespace thunderbird::calib
