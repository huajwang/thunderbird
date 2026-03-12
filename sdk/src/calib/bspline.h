// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Uniform Cubic B-Spline
// ─────────────────────────────────────────────────────────────────────────────
//
// Lightweight uniform cubic (order 4) B-spline for trajectory interpolation.
// Primary use: smooth IMU interpolation at arbitrary timestamps, replacing
// linear interpolation for smoother deskewing.
//
// The implementation is self-contained (no Eigen dependency). The basis
// matrix is the standard uniform cubic B-spline matrix from:
//   Qin, K. "General Matrix Representations for B-splines" (2000)
//
// Inspired by Kalibr's B-spline library (ETH Zurich, BSD 3-Clause) but
// reimplemented from the mathematical definition for Thunderbird's needs.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <vector>

namespace thunderbird::calib {

/// Uniform cubic B-spline with vector-valued coefficients.
///
/// Supports:
///  - Fitting to timestamped data (uniform knot spacing)
///  - Evaluation at arbitrary time t within the valid interval
///  - First and second derivatives (velocity, acceleration)
///
/// Template-free: coefficient dimension is set at construction time.
class UniformCubicBSpline {
public:
    /// Construct a spline with the given coefficient dimension.
    /// @param dim  Number of components per control point (e.g. 3 for 3D, 6 for IMU).
    explicit UniformCubicBSpline(int dim);

    /// Initialize spline from uniformly sampled data.
    ///
    /// Creates control points such that the spline passes near the data.
    /// Control points are placed at uniform spacing dt = (t1 - t0) / (n - 1)
    /// where n = data.size(). The spline is valid on [t0 + dt, t1 - dt]
    /// (one segment inward on each side due to cubic support width).
    ///
    /// @param t0        Timestamp of the first data sample.
    /// @param t1        Timestamp of the last data sample.
    /// @param data      Row-major data: data[i * dim + j] is sample i, component j.
    /// @param n_samples Number of data samples.
    /// @return true if initialization succeeded.
    bool initFromData(double t0, double t1,
                      const double* data, int n_samples);

    /// Initialize spline from explicit control points and time range.
    ///
    /// @param t0     Start time of the knot sequence.
    /// @param dt     Uniform knot spacing.
    /// @param coeffs Control point values, row-major: coeffs[i * dim + j].
    /// @param n_ctrl Number of control points (must be >= 4).
    /// @return true if initialization succeeded.
    bool initFromCoefficients(double t0, double dt,
                              const double* coeffs, int n_ctrl);

    /// Evaluate the spline at time t.
    /// @param t       Evaluation time (must be within valid interval).
    /// @param result  Output buffer of size dim().
    /// @return true if t is within the valid interval; false otherwise.
    bool eval(double t, double* result) const;

    /// Evaluate the d-th derivative at time t.
    /// @param t       Evaluation time.
    /// @param order   Derivative order (0 = value, 1 = velocity, 2 = acceleration).
    /// @param result  Output buffer of size dim().
    /// @return true if t is within the valid interval.
    bool evalDerivative(double t, int order, double* result) const;

    /// Valid time interval [t_min, t_max].
    double tMin() const { return t0_ + dt_; }
    double tMax() const { return t0_ + (static_cast<double>(n_ctrl_ - 2)) * dt_; }

    /// Coefficient dimension.
    int dim() const { return dim_; }

    /// Number of control points.
    int numControlPoints() const { return n_ctrl_; }

    /// Whether the spline has been initialized.
    bool isValid() const { return n_ctrl_ >= 4; }

private:
    /// Compute segment index and local parameter u in [0,1).
    bool segmentParam(double t, int& seg, double& u) const;

    int    dim_;
    double t0_ = 0.0;   ///< Start of knot sequence.
    double dt_ = 1.0;   ///< Uniform knot spacing.
    int    n_ctrl_ = 0;  ///< Number of control points.

    /// Flat control point storage: coeffs_[i * dim_ + j].
    std::vector<double> coeffs_;
};

}  // namespace thunderbird::calib
