// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Evaluation Framework: IMU Interpolator
// ─────────────────────────────────────────────────────────────────────────────
//
// Synthesises IMU samples from ground truth poses by numerical differentiation.
// Used when a dataset has LiDAR but no native IMU data (e.g. KITTI Odometry).
//
// Strategy:
//   Given GT poses (position + quaternion) at LiDAR timestamps t0, t1, t2…
//   1. Cubic Hermite interpolation of position at configurable IMU rate
//      (default 200 Hz).
//   2. Finite-difference 2nd derivative → acceleration in body frame.
//   3. SLERP of quaternions → angular velocity via log-map differentiation.
//   4. Optional additive Gaussian noise (configurable σ for accel, gyro).
//   5. Gravity vector added to acceleration (body frame).
//
// Buffering contract:
//   - Call prepare(gt_poses) once with the full ground truth.
//   - Call generateBetween(t_start_ns, t_end_ns) to get all synthetic IMU
//     samples strictly in (t_start, t_end].  This is called between each
//     pair of consecutive LiDAR scans so the engine receives IMU before
//     the corresponding point cloud.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "dataset_adapter.h"

#include <array>
#include <cstdint>
#include <random>
#include <vector>

namespace eval {

// ═════════════════════════════════════════════════════════════════════════════
//  ImuInterpolator configuration
// ═════════════════════════════════════════════════════════════════════════════

struct ImuInterpolatorConfig {
    double imu_rate_hz{200.0};           ///< Synthetic IMU generation rate
    double accel_noise_sigma{0.01};      ///< m/s², std-dev of additive noise
    double gyro_noise_sigma{0.001};      ///< rad/s, std-dev of additive noise
    std::array<double,3> gravity{0.0, 0.0, -9.81};  ///< world-frame gravity
    bool   add_noise{true};              ///< Enable noise injection
    uint64_t noise_seed{42};             ///< RNG seed for reproducibility
};

// ═════════════════════════════════════════════════════════════════════════════
//  ImuInterpolator
// ═════════════════════════════════════════════════════════════════════════════

class ImuInterpolator {
public:
    explicit ImuInterpolator(ImuInterpolatorConfig cfg = {});

    /// Ingest GT poses (must be sorted by timestamp_ns, minimum 2 poses).
    /// Call once before any generateBetween() calls.
    void prepare(const std::vector<GtPose>& gt_poses);

    /// Generate synthetic IMU samples in the half-open interval (t_start, t_end].
    /// Returns samples in strict timestamp order at the configured rate.
    std::vector<ImuSample> generateBetween(int64_t t_start_ns, int64_t t_end_ns);

    /// Generate all IMU samples spanning the full GT trajectory.
    std::vector<ImuSample> generateAll();

    /// Number of GT poses loaded.
    size_t numPoses() const { return poses_.size(); }

    /// IMU rate.
    double rateHz() const { return cfg_.imu_rate_hz; }

private:
    ImuInterpolatorConfig cfg_;
    std::vector<GtPose>   poses_;
    std::mt19937          rng_;

    // ── Interpolation helpers ───────────────────────────────────────────

    /// Interpolate position at time t_ns using cubic Hermite spline
    /// between the two nearest GT poses.
    std::array<double,3> interpPosition(int64_t t_ns) const;

    /// Interpolate quaternion at time t_ns using SLERP.
    std::array<double,4> interpQuaternion(int64_t t_ns) const;

    /// Compute acceleration at time t_ns by central finite difference
    /// of interpolated position, then rotate into body frame.
    std::array<double,3> computeAccel(int64_t t_ns) const;

    /// Compute angular velocity at time t_ns via quaternion log-map
    /// differentiation of interpolated rotation.
    std::array<double,3> computeGyro(int64_t t_ns) const;

    /// Find the index of the GT pose just before t_ns.
    size_t findSegment(int64_t t_ns) const;

    /// Quaternion multiplication: q1 * q2 (Hamilton convention, [w,x,y,z]).
    static std::array<double,4> quatMul(const std::array<double,4>& q1,
                                         const std::array<double,4>& q2);

    /// Quaternion conjugate (inverse for unit quaternion).
    static std::array<double,4> quatConj(const std::array<double,4>& q);

    /// Normalise quaternion in-place.
    static void quatNormalize(std::array<double,4>& q);

    /// SLERP between two quaternions.
    static std::array<double,4> slerp(const std::array<double,4>& q0,
                                       const std::array<double,4>& q1,
                                       double t);

    /// Rotate vector by quaternion: q * [0,v] * q^{-1}.
    static std::array<double,3> rotateByQuat(const std::array<double,4>& q,
                                              const std::array<double,3>& v);
};

} // namespace eval
