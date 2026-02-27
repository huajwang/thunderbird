// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Evaluation Framework: IMU Interpolator Implementation
// ─────────────────────────────────────────────────────────────────────────────

#include "eval/imu_interpolator.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <numeric>

namespace eval {

// ═════════════════════════════════════════════════════════════════════════════
//  Construction
// ═════════════════════════════════════════════════════════════════════════════

ImuInterpolator::ImuInterpolator(ImuInterpolatorConfig cfg)
    : cfg_(cfg), rng_(cfg.noise_seed) {}

// ═════════════════════════════════════════════════════════════════════════════
//  prepare()
// ═════════════════════════════════════════════════════════════════════════════

void ImuInterpolator::prepare(const std::vector<GtPose>& gt_poses) {
    poses_ = gt_poses;
    // Ensure sorted by timestamp.
    std::sort(poses_.begin(), poses_.end(),
        [](const GtPose& a, const GtPose& b) {
            return a.timestamp_ns < b.timestamp_ns;
        });

    if (poses_.size() < 2) {
        std::cerr << "[imu_interp] warning: need ≥2 GT poses, got "
                  << poses_.size() << "\n";
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  generateBetween() — core buffering interface
// ═════════════════════════════════════════════════════════════════════════════

std::vector<ImuSample> ImuInterpolator::generateBetween(
    int64_t t_start_ns, int64_t t_end_ns)
{
    std::vector<ImuSample> samples;
    if (poses_.size() < 2 || t_start_ns >= t_end_ns) return samples;

    // Clamp to GT time span (with small margin).
    const int64_t gt_start = poses_.front().timestamp_ns;
    const int64_t gt_end   = poses_.back().timestamp_ns;
    const int64_t eff_start = std::max(t_start_ns, gt_start);
    const int64_t eff_end   = std::min(t_end_ns, gt_end);
    if (eff_start >= eff_end) return samples;

    // IMU sample period in nanoseconds.
    const int64_t period_ns = static_cast<int64_t>(
        std::llround(1.0e9 / cfg_.imu_rate_hz));

    // First IMU timestamp: smallest multiple of period_ns that is > t_start_ns.
    int64_t t = eff_start + period_ns - (eff_start % period_ns);
    if (t <= eff_start) t += period_ns;

    std::normal_distribution<double> accel_noise(0.0, cfg_.accel_noise_sigma);
    std::normal_distribution<double> gyro_noise(0.0, cfg_.gyro_noise_sigma);

    // Small dt for finite differences (1 ms).
    constexpr int64_t FD_DT_NS = 1'000'000;

    while (t <= eff_end) {
        ImuSample s;
        s.timestamp_ns = t;

        // ── Acceleration via central finite difference ──────────────────
        s.accel = computeAccel(t);

        // ── Angular velocity via quaternion differentiation ─────────────
        s.gyro = computeGyro(t);

        // ── Add noise ───────────────────────────────────────────────────
        if (cfg_.add_noise) {
            for (int i = 0; i < 3; ++i) {
                s.accel[i] += accel_noise(rng_);
                s.gyro[i]  += gyro_noise(rng_);
            }
        }

        s.temperature = 25.0;  // synthetic, nominal
        samples.push_back(s);

        t += period_ns;
    }

    return samples;
}

// ═════════════════════════════════════════════════════════════════════════════
//  generateAll()
// ═════════════════════════════════════════════════════════════════════════════

std::vector<ImuSample> ImuInterpolator::generateAll() {
    if (poses_.size() < 2) return {};
    return generateBetween(poses_.front().timestamp_ns, poses_.back().timestamp_ns);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Segment finder — binary search for the GT interval containing t_ns
// ═════════════════════════════════════════════════════════════════════════════

size_t ImuInterpolator::findSegment(int64_t t_ns) const {
    // Returns index i such that poses_[i].timestamp_ns ≤ t_ns < poses_[i+1].timestamp_ns.
    auto it = std::upper_bound(poses_.begin(), poses_.end(), t_ns,
        [](int64_t t, const GtPose& p) { return t < p.timestamp_ns; });
    if (it == poses_.begin()) return 0;
    size_t idx = static_cast<size_t>(it - poses_.begin()) - 1;
    if (idx >= poses_.size() - 1) idx = poses_.size() - 2;
    return idx;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Position interpolation — cubic Hermite
// ═════════════════════════════════════════════════════════════════════════════

std::array<double,3> ImuInterpolator::interpPosition(int64_t t_ns) const {
    const size_t i = findSegment(t_ns);
    const auto& p0 = poses_[i];
    const auto& p1 = poses_[i + 1];

    const double dt = static_cast<double>(p1.timestamp_ns - p0.timestamp_ns);
    if (dt <= 0.0) return p0.position;

    const double u = static_cast<double>(t_ns - p0.timestamp_ns) / dt;

    // Estimate tangents from neighbours (Catmull-Rom style).
    auto tangent = [&](size_t idx) -> std::array<double,3> {
        std::array<double,3> m{};
        if (idx > 0 && idx + 1 < poses_.size()) {
            double dt_span = static_cast<double>(
                poses_[idx+1].timestamp_ns - poses_[idx-1].timestamp_ns) * 1.0e-9;
            if (dt_span > 0) {
                for (int k = 0; k < 3; ++k) {
                    m[k] = (poses_[idx+1].position[k] - poses_[idx-1].position[k]) / dt_span;
                }
            }
        } else if (idx + 1 < poses_.size()) {
            double dt_seg = static_cast<double>(
                poses_[idx+1].timestamp_ns - poses_[idx].timestamp_ns) * 1.0e-9;
            if (dt_seg > 0) {
                for (int k = 0; k < 3; ++k)
                    m[k] = (poses_[idx+1].position[k] - poses_[idx].position[k]) / dt_seg;
            }
        } else if (idx > 0) {
            double dt_seg = static_cast<double>(
                poses_[idx].timestamp_ns - poses_[idx-1].timestamp_ns) * 1.0e-9;
            if (dt_seg > 0) {
                for (int k = 0; k < 3; ++k)
                    m[k] = (poses_[idx].position[k] - poses_[idx-1].position[k]) / dt_seg;
            }
        }
        return m;
    };

    const auto m0 = tangent(i);
    const auto m1 = tangent(i + 1);
    const double dt_s = dt * 1.0e-9;

    // Hermite basis functions.
    const double u2 = u * u, u3 = u2 * u;
    const double h00 = 2*u3 - 3*u2 + 1;
    const double h10 = u3 - 2*u2 + u;
    const double h01 = -2*u3 + 3*u2;
    const double h11 = u3 - u2;

    std::array<double,3> result{};
    for (int k = 0; k < 3; ++k) {
        result[k] = h00 * p0.position[k]
                  + h10 * dt_s * m0[k]
                  + h01 * p1.position[k]
                  + h11 * dt_s * m1[k];
    }
    return result;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Quaternion interpolation — SLERP
// ═════════════════════════════════════════════════════════════════════════════

std::array<double,4> ImuInterpolator::interpQuaternion(int64_t t_ns) const {
    const size_t i = findSegment(t_ns);
    const auto& p0 = poses_[i];
    const auto& p1 = poses_[i + 1];

    const double dt = static_cast<double>(p1.timestamp_ns - p0.timestamp_ns);
    if (dt <= 0.0) return p0.quaternion;

    const double u = static_cast<double>(t_ns - p0.timestamp_ns) / dt;
    return slerp(p0.quaternion, p1.quaternion, u);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Acceleration — 2nd derivative of position + gravity in body frame
// ═════════════════════════════════════════════════════════════════════════════

std::array<double,3> ImuInterpolator::computeAccel(int64_t t_ns) const {
    // Central finite difference with 1 ms step.
    constexpr int64_t h = 1'000'000;  // 1 ms in ns

    const int64_t t_lo = std::max(t_ns - h, poses_.front().timestamp_ns);
    const int64_t t_hi = std::min(t_ns + h, poses_.back().timestamp_ns);
    const int64_t t_mid = t_ns;

    const auto p_lo  = interpPosition(t_lo);
    const auto p_mid = interpPosition(t_mid);
    const auto p_hi  = interpPosition(t_hi);

    const double dt_lo = static_cast<double>(t_mid - t_lo) * 1.0e-9;
    const double dt_hi = static_cast<double>(t_hi - t_mid) * 1.0e-9;
    const double dt_total = dt_lo + dt_hi;

    std::array<double,3> accel_world{};
    if (dt_total > 0.0 && dt_lo > 0.0 && dt_hi > 0.0) {
        for (int k = 0; k < 3; ++k) {
            // Second derivative: (p_hi - 2*p_mid + p_lo) / (dt/2)^2
            double d2 = (p_hi[k] - 2.0*p_mid[k] + p_lo[k]);
            double dt_avg = (dt_lo + dt_hi) / 2.0;
            accel_world[k] = d2 / (dt_avg * dt_avg);
        }
    }

    // Subtract world-frame gravity to get specific force (what IMU measures):
    // a_imu = R^T * (a_world - g_world)
    std::array<double,3> specific_force_world{};
    for (int k = 0; k < 3; ++k) {
        specific_force_world[k] = accel_world[k] - cfg_.gravity[k];
    }

    // Rotate to body frame.
    auto q = interpQuaternion(t_ns);
    auto q_inv = quatConj(q);
    return rotateByQuat(q_inv, specific_force_world);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Angular velocity — quaternion finite difference
// ═════════════════════════════════════════════════════════════════════════════

std::array<double,3> ImuInterpolator::computeGyro(int64_t t_ns) const {
    constexpr int64_t h = 1'000'000;  // 1 ms

    const int64_t t_lo = std::max(t_ns - h, poses_.front().timestamp_ns);
    const int64_t t_hi = std::min(t_ns + h, poses_.back().timestamp_ns);

    const auto q_lo = interpQuaternion(t_lo);
    const auto q_hi = interpQuaternion(t_hi);

    const double dt = static_cast<double>(t_hi - t_lo) * 1.0e-9;
    if (dt <= 0.0) return {0, 0, 0};

    // dq = q_hi * q_lo^{-1}  →  angular velocity ≈ 2 * vec(dq) / dt
    auto q_lo_inv = quatConj(q_lo);
    auto dq = quatMul(q_hi, q_lo_inv);

    // Ensure shortest path.
    if (dq[0] < 0.0) {
        for (auto& c : dq) c = -c;
    }

    // Angular velocity in world frame.
    std::array<double,3> omega_world{};
    omega_world[0] = 2.0 * dq[1] / dt;
    omega_world[1] = 2.0 * dq[2] / dt;
    omega_world[2] = 2.0 * dq[3] / dt;

    // Rotate to body frame: omega_body = R^T * omega_world
    auto q_mid = interpQuaternion(t_ns);
    auto q_mid_inv = quatConj(q_mid);
    return rotateByQuat(q_mid_inv, omega_world);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Quaternion utilities
// ═════════════════════════════════════════════════════════════════════════════

std::array<double,4> ImuInterpolator::quatMul(
    const std::array<double,4>& a, const std::array<double,4>& b)
{
    // Hamilton convention: [w,x,y,z]
    return {
        a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
        a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
        a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
        a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]
    };
}

std::array<double,4> ImuInterpolator::quatConj(const std::array<double,4>& q) {
    return {q[0], -q[1], -q[2], -q[3]};
}

void ImuInterpolator::quatNormalize(std::array<double,4>& q) {
    double n = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n > 0.0) {
        for (auto& c : q) c /= n;
    } else {
        q = {1.0, 0.0, 0.0, 0.0};  // degenerate → identity
    }
}

std::array<double,4> ImuInterpolator::slerp(
    const std::array<double,4>& q0_in,
    const std::array<double,4>& q1_in,
    double t)
{
    auto q0 = q0_in;
    auto q1 = q1_in;

    double dot = q0[0]*q1[0] + q0[1]*q1[1] + q0[2]*q1[2] + q0[3]*q1[3];

    // Ensure shortest arc.
    if (dot < 0.0) {
        for (auto& c : q1) c = -c;
        dot = -dot;
    }

    // If very close, use linear interpolation to avoid division by zero.
    if (dot > 0.9995) {
        std::array<double,4> result{};
        for (int i = 0; i < 4; ++i)
            result[i] = q0[i] + t * (q1[i] - q0[i]);
        quatNormalize(result);
        return result;
    }

    double theta = std::acos(std::clamp(dot, -1.0, 1.0));
    double sin_theta = std::sin(theta);

    double w0 = std::sin((1.0 - t) * theta) / sin_theta;
    double w1 = std::sin(t * theta) / sin_theta;

    std::array<double,4> result{};
    for (int i = 0; i < 4; ++i)
        result[i] = w0 * q0[i] + w1 * q1[i];
    quatNormalize(result);
    return result;
}

std::array<double,3> ImuInterpolator::rotateByQuat(
    const std::array<double,4>& q, const std::array<double,3>& v)
{
    // q * [0,v] * q^{-1}   (for unit quaternion, conj = inverse)
    std::array<double,4> qv = {0.0, v[0], v[1], v[2]};
    auto tmp = quatMul(q, qv);
    auto result = quatMul(tmp, quatConj(q));
    return {result[1], result[2], result[3]};
}

} // namespace eval
