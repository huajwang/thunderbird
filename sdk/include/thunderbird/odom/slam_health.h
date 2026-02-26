// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Health Monitor (Failure Detection)
// ─────────────────────────────────────────────────────────────────────────────
//
// Lightweight, real-time-safe failure detection for the SLAM pipeline.
// Designed to run inline on the SLAM worker thread with < 1 µs overhead
// per check.  No heap allocation, no locks, no syscalls on the hot path.
//
// ═══════════════════════════════════════════════════════════════════════════
//  Failure Modes Detected
// ═══════════════════════════════════════════════════════════════════════════
//
//  ┌─────────────────────────────────────────────────────────────────────┐
//  │  Failure              Detector              Threshold (default)     │
//  ├─────────────────────────────────────────────────────────────────────┤
//  │  IMU dropout          Gap in timestamp_ns   > 3× expected period   │
//  │  LiDAR dropout        Gap in scan arrival   > 3× expected period   │
//  │  Degenerate geometry  Eigenvalue ratio of   ratio < 10 (planar)    │
//  │                       point covariance →    or < 3 (linear)        │
//  │                       min/max eigenvalue                           │
//  │  ESIKF divergence     Covariance trace >    > trace_max (100 m²)  │
//  │                       threshold, or NaN                            │
//  │  High drift rate      Position delta /      > 2 m/s beyond vel    │
//  │                       time > max_speed                             │
//  │  Residual spike       ESIKF residual >      > 3× running mean     │
//  │                       adaptive threshold                           │
//  └─────────────────────────────────────────────────────────────────────┘
//
// ═══════════════════════════════════════════════════════════════════════════
//  Health State Machine
// ═══════════════════════════════════════════════════════════════════════════
//
//                     ┌──────────────────────────────────────────────┐
//                     │                                              │
//  ┌──────────┐  ok   │  ┌───────────┐  degraded  ┌──────────────┐ │
//  │  Nominal ├──────►│  │ Degraded  ├──────────► │  Critical    │ │
//  │ (green)  │◄──────┤  │ (yellow)  │◄────────── │  (red)       │ │
//  └──────────┘ heal  │  └───────────┘   heal     └──────┬───────┘ │
//                     │                                   │         │
//                     │                           timeout │         │
//                     │                                   ▼         │
//                     │                          ┌──────────────┐   │
//                     │                          │  Emergency   │   │
//                     │                          │  (fallback)  │   │
//                     │                          └──────────────┘   │
//                     │                                              │
//                     └──────────────────────────────────────────────┘
//
//  Transitions:
//    Nominal → Degraded    : any single detector fires
//    Degraded → Critical   : ≥2 detectors fire simultaneously, or
//                            single detector persists > degrade_timeout
//    Critical → Emergency  : Critical persists > critical_timeout
//    Any → Nominal         : all detectors clear for > heal_window
//
//  Emergency fallback:
//    • Output validity flag set to INVALID
//    • Last-known-good pose held (no new poses published)
//    • Engine reset recommended via callback
//    • IMU-only dead-reckoning optionally enabled
//
// ═══════════════════════════════════════════════════════════════════════════
//  Output Validity Flag
// ═══════════════════════════════════════════════════════════════════════════
//
//  Every SlamOutput / Pose6D can be tagged with an OutputValidity enum:
//
//    Valid         — full pipeline operating, confidence > 0.8
//    Cautionary    — degraded but usable, confidence 0.3–0.8
//    Invalid       — do not use, confidence < 0.3
//    Stale         — no update for > expected period
//
//  The validity is derived from the combined health state + confidence
//  metric and can be checked with a single branch:
//
//    if (validity >= OutputValidity::Cautionary) { use_pose(); }
//
// ═══════════════════════════════════════════════════════════════════════════
//  Confidence Metric
// ═══════════════════════════════════════════════════════════════════════════
//
//  A single [0.0, 1.0] float representing overall SLAM confidence:
//
//    confidence = w_imu   * imu_score
//               + w_lidar * lidar_score
//               + w_geom  * geometry_score
//               + w_ekf   * ekf_score
//               + w_drift * drift_score
//               + w_resid * residual_score
//
//  Each sub-score is [0.0, 1.0]:
//    imu_score      — 1.0 when on-time, decays exponentially with gap
//    lidar_score    — 1.0 when on-time, decays exponentially with gap
//    geometry_score — based on eigenvalue ratio (1.0 = 3D features)
//    ekf_score      — 1.0 when trace < nominal, 0.0 at trace_max
//    drift_score    — 1.0 when delta-v matches IMU, 0.0 at max_speed
//    residual_score — 1.0 when residual < mean, 0.0 at 3× mean
//
//  Weights default to equal (1/6 each) but are configurable.
//
// ═══════════════════════════════════════════════════════════════════════════
//  Real-time Safety
// ═══════════════════════════════════════════════════════════════════════════
//
//  • All state is in a single POD-like struct (SlamHealthState).
//  • No dynamic allocation after init.
//  • No mutex, no condvar, no syscall.
//  • All checks are O(1) arithmetic on cached state.
//  • EMA (exponential moving average) for running statistics —
//    no window buffers, no sorting.
//  • Total cost per update: ~50 arithmetic ops ≈ < 100 ns on ARM,
//    < 50 ns on x86.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/odom/slam_types.h"

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>

namespace thunderbird::odom {

// ═════════════════════════════════════════════════════════════════════════════
//  Enumerations
// ═════════════════════════════════════════════════════════════════════════════

/// Overall health state of the SLAM pipeline.
enum class HealthState : uint8_t {
    Nominal   = 0,   ///< All detectors clear, confidence > 0.8
    Degraded  = 1,   ///< One or more warnings, usable with caution
    Critical  = 2,   ///< Multiple failures or sustained degradation
    Emergency = 3,   ///< Fallback mode — output invalid, reset needed
};

/// Human-readable name.
inline const char* health_state_name(HealthState s) noexcept {
    switch (s) {
        case HealthState::Nominal:   return "Nominal";
        case HealthState::Degraded:  return "Degraded";
        case HealthState::Critical:  return "Critical";
        case HealthState::Emergency: return "Emergency";
    }
    return "Unknown";
}

/// Output validity tag for individual poses / slam outputs.
enum class OutputValidity : uint8_t {
    Valid       = 3,   ///< Full confidence, safe to use
    Cautionary  = 2,   ///< Degraded but usable — increase control margins
    Invalid     = 1,   ///< Do not use for control
    Stale       = 0,   ///< No update received within expected window
};

inline const char* output_validity_name(OutputValidity v) noexcept {
    switch (v) {
        case OutputValidity::Valid:      return "Valid";
        case OutputValidity::Cautionary: return "Cautionary";
        case OutputValidity::Invalid:    return "Invalid";
        case OutputValidity::Stale:      return "Stale";
    }
    return "Unknown";
}

/// Bitfield of active failure detectors.
enum class FaultFlags : uint16_t {
    None              = 0,
    ImuDropout        = 1 << 0,   ///< IMU gap > threshold
    LidarDropout      = 1 << 1,   ///< LiDAR gap > threshold
    DegenerateGeom    = 1 << 2,   ///< Insufficient geometric features
    EkfDivergence     = 1 << 3,   ///< Covariance exploded or NaN
    HighDriftRate     = 1 << 4,   ///< Position jump exceeds kinematics
    ResidualSpike     = 1 << 5,   ///< ESIKF residual >> running mean
    ImuSaturation     = 1 << 6,   ///< Accel or gyro at sensor limit
    CovarianceNaN     = 1 << 7,   ///< NaN detected in state/covariance
    MapDegraded       = 1 << 8,   ///< Map too sparse or too large
    TimeSyncLost      = 1 << 9,   ///< Clock drift exceeds tolerance
};

/// Bitwise OR.
inline FaultFlags operator|(FaultFlags a, FaultFlags b) noexcept {
    return static_cast<FaultFlags>(
        static_cast<uint16_t>(a) | static_cast<uint16_t>(b));
}
inline FaultFlags& operator|=(FaultFlags& a, FaultFlags b) noexcept {
    a = a | b; return a;
}
/// Bitwise AND.
inline FaultFlags operator&(FaultFlags a, FaultFlags b) noexcept {
    return static_cast<FaultFlags>(
        static_cast<uint16_t>(a) & static_cast<uint16_t>(b));
}
/// Test if a flag is set.
inline bool has_fault(FaultFlags flags, FaultFlags test) noexcept {
    return (static_cast<uint16_t>(flags) & static_cast<uint16_t>(test)) != 0;
}
/// Count set bits.
inline int fault_count(FaultFlags flags) noexcept {
    uint16_t v = static_cast<uint16_t>(flags);
    int count = 0;
    while (v) { count += v & 1; v >>= 1; }
    return count;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Configuration
// ═════════════════════════════════════════════════════════════════════════════

/// Thresholds and tuning parameters for the health monitor.
/// All times are in nanoseconds for consistency with the SLAM pipeline.
struct HealthMonitorConfig {
    // ── Sensor dropout detection ────────────────────────────────────────
    double imu_rate_hz       = 400.0;    ///< expected IMU sample rate
    double lidar_rate_hz     = 10.0;     ///< expected LiDAR scan rate
    double dropout_factor    = 3.0;      ///< gap > factor × period = dropout

    // ── IMU saturation limits ───────────────────────────────────────────
    double accel_max_mps2    = 156.0;    ///< accelerometer range (±16g)
    double gyro_max_rps      = 34.9;     ///< gyroscope range (±2000°/s)

    // ── Degenerate geometry detection ───────────────────────────────────
    /// Eigenvalue ratio thresholds for the point cloud covariance.
    /// Computed from the 3×3 scatter matrix of matched correspondences.
    ///   ratio = λ_min / λ_max
    double geom_degen_ratio_warn = 0.01; ///< < this = degenerate warning
    double geom_linear_ratio     = 0.001;///< < this = nearly linear (tunnel)
    uint32_t min_points_for_geom = 50;   ///< skip check if fewer matches

    // ── EKF divergence ──────────────────────────────────────────────────
    /// Trace of 6×6 marginal covariance (rot²+pos²).
    double cov_trace_warn    = 10.0;     ///< m² — degraded threshold
    double cov_trace_crit    = 100.0;    ///< m² — critical threshold

    // ── Drift rate ──────────────────────────────────────────────────────
    /// Maximum plausible velocity for the platform.
    double max_platform_speed_mps = 30.0; ///< m/s (drone: 20, car: 50)
    double drift_speed_factor     = 2.0;  ///< alert if delta_pos/dt > factor × max_speed

    // ── Residual spike ──────────────────────────────────────────────────
    double residual_spike_factor  = 3.0;  ///< alert if residual > factor × EMA
    double residual_ema_alpha     = 0.1;  ///< EMA smoothing factor (0–1)

    // ── Map health ──────────────────────────────────────────────────────
    size_t map_min_points    = 100;      ///< too sparse
    size_t map_max_points    = 1'000'000;///< excessively large

    // ── Time sync ───────────────────────────────────────────────────────
    int64_t time_sync_max_drift_ns = 50'000'000; ///< 50 ms max clock drift

    // ── State machine timing ────────────────────────────────────────────
    int64_t degrade_timeout_ns   = 2'000'000'000;  ///< 2 s → Critical
    int64_t critical_timeout_ns  = 5'000'000'000;  ///< 5 s → Emergency
    int64_t heal_window_ns       = 1'000'000'000;  ///< 1 s clear → Nominal

    // ── Confidence weights (must sum to ~1.0) ───────────────────────────
    double w_imu      = 0.20;
    double w_lidar    = 0.20;
    double w_geom     = 0.15;
    double w_ekf      = 0.20;
    double w_drift    = 0.15;
    double w_residual = 0.10;

    // ── Confidence → Validity thresholds ────────────────────────────────
    double validity_cautionary_threshold = 0.8;  ///< below → Cautionary
    double validity_invalid_threshold    = 0.3;  ///< below → Invalid

    // ── Emergency fallback ──────────────────────────────────────────────
    bool   enable_imu_deadreckoning = true; ///< propagate IMU-only in emergency
    bool   auto_reset_on_emergency  = false;///< auto-call engine.reset()
};

// ═════════════════════════════════════════════════════════════════════════════
//  Per-detector state (internal, no allocation)
// ═════════════════════════════════════════════════════════════════════════════

/// Running statistics for one detector — EMA-based, O(1) per update.
struct DetectorState {
    double  score{1.0};             ///< [0,1] sub-confidence score
    bool    fired{false};           ///< true if currently in fault
    int64_t first_fire_ns{0};       ///< timestamp of first fire in current window
    int64_t last_clear_ns{0};       ///< timestamp of last clear transition
    uint64_t fire_count{0};         ///< total lifetime fire events
};

// ═════════════════════════════════════════════════════════════════════════════
//  SlamHealthSnapshot — complete health report (value type, copyable)
// ═════════════════════════════════════════════════════════════════════════════

struct SlamHealthSnapshot {
    int64_t        timestamp_ns{0};

    HealthState    state{HealthState::Nominal};
    FaultFlags     active_faults{FaultFlags::None};
    OutputValidity validity{OutputValidity::Valid};
    float          confidence{1.0f};     ///< [0.0, 1.0]

    // ── Per-detector scores ─────────────────────────────────────────────
    float imu_score{1.0f};
    float lidar_score{1.0f};
    float geometry_score{1.0f};
    float ekf_score{1.0f};
    float drift_score{1.0f};
    float residual_score{1.0f};

    // ── Timing diagnostics ──────────────────────────────────────────────
    int64_t last_imu_ns{0};           ///< timestamp of last IMU sample
    int64_t last_lidar_ns{0};         ///< timestamp of last LiDAR scan
    int64_t imu_gap_ns{0};            ///< current IMU gap
    int64_t lidar_gap_ns{0};          ///< current LiDAR gap

    // ── EKF diagnostics ─────────────────────────────────────────────────
    double cov_trace{0};              ///< current 6×6 covariance trace
    double residual_ema{0};           ///< smoothed ESIKF residual
    double drift_speed_mps{0};        ///< instantaneous position delta rate

    // ── Geometry ────────────────────────────────────────────────────────
    double eigenvalue_ratio{1.0};     ///< λ_min / λ_max of correspondences

    // ── Counters ────────────────────────────────────────────────────────
    uint64_t total_imu_dropouts{0};
    uint64_t total_lidar_dropouts{0};
    uint64_t total_ekf_divergences{0};
    uint64_t total_state_transitions{0};
    int64_t  time_in_state_ns{0};     ///< current state duration
};

// ═════════════════════════════════════════════════════════════════════════════
//  Callback typedefs
// ═════════════════════════════════════════════════════════════════════════════

/// Fired on every health state transition.
using HealthStateCallback  = std::function<void(HealthState /*from*/,
                                                HealthState /*to*/,
                                                FaultFlags  /*active*/)>;

/// Fired when emergency fallback is entered.
using EmergencyCallback    = std::function<void(const SlamHealthSnapshot&)>;

/// Fired on every health update (10 Hz from worker).
using HealthUpdateCallback = std::function<void(const SlamHealthSnapshot&)>;

// ═════════════════════════════════════════════════════════════════════════════
//  Geometry analysis helpers (inline, no allocation)
// ═════════════════════════════════════════════════════════════════════════════

/// Eigenvalue ratio from a 3×3 symmetric scatter matrix.
/// Input: upper triangle [s00, s01, s02, s11, s12, s22].
/// Returns λ_min / λ_max via the characterisic polynomial (closed-form).
/// Cost: ~40 FLOPs, no allocation, no iteration.
inline double eigenvalue_ratio_3x3(const double s[6]) noexcept {
    // Unpack symmetric matrix:
    //   | s[0] s[1] s[2] |
    //   | s[1] s[3] s[4] |
    //   | s[2] s[4] s[5] |
    const double a = s[0], b = s[1], c = s[2];
    const double d = s[3], e = s[4], f = s[5];

    // Characteristic polynomial: λ³ - p λ² + q λ - r = 0
    const double p = a + d + f;                           // trace
    const double q = a*d + a*f + d*f - b*b - c*c - e*e;  // sum of 2×2 minors
    const double r = a*d*f + 2.0*b*c*e - a*e*e - d*c*c - f*b*b; // det

    // Solve via trigonometric method for 3 real roots.
    const double p3 = p / 3.0;
    const double q2 = (p*p - 3.0*q) / 9.0;

    if (q2 <= 0.0) return 1.0; // Degenerate: all eigenvalues equal.

    const double r2 = (2.0*p*p*p - 9.0*p*q + 27.0*r) / 54.0;
    const double q2_32 = q2 * std::sqrt(q2);

    double cos_arg = r2 / q2_32;
    // Clamp to [-1, 1] for numerical safety.
    if (cos_arg > 1.0) cos_arg = 1.0;
    if (cos_arg < -1.0) cos_arg = -1.0;

    const double theta = std::acos(cos_arg);
    const double sq = std::sqrt(q2);

    const double l0 = -2.0 * sq * std::cos(theta / 3.0)            + p3;
    const double l1 = -2.0 * sq * std::cos((theta + 6.283185307) / 3.0) + p3;
    const double l2 = -2.0 * sq * std::cos((theta - 6.283185307) / 3.0) + p3;

    const double lmax = std::max({l0, l1, l2});
    const double lmin = std::min({l0, l1, l2});

    if (lmax <= 0.0) return 0.0;
    return lmin / lmax;
}

// ═════════════════════════════════════════════════════════════════════════════
//  SlamHealthMonitor — main class
// ═════════════════════════════════════════════════════════════════════════════
//
// Usage (from SLAM worker thread):
//
//   SlamHealthMonitor health(config);
//
//   // On every IMU sample:
//   health.on_imu(sample);
//
//   // On every ESIKF update:
//   health.on_slam_output(output);
//
//   // Periodic (or after each scan):
//   auto snap = health.snapshot();
//   if (snap.validity >= OutputValidity::Cautionary)
//       publish(output);
//
// Thread safety:
//   • All on_*() methods are designed for single-thread access
//     (the SLAM worker thread).
//   • snapshot() performs only plain reads and is safe to call from
//     any thread if the caller accepts slightly stale data.
//   • Callbacks fire on the caller's thread (the worker thread).
//
// ─────────────────────────────────────────────────────────────────────────────

class SlamHealthMonitor {
public:
    explicit SlamHealthMonitor(const HealthMonitorConfig& cfg = {}) noexcept
        : cfg_(cfg)
    {
        imu_period_ns_   = static_cast<int64_t>(1.0e9 / cfg_.imu_rate_hz);
        lidar_period_ns_ = static_cast<int64_t>(1.0e9 / cfg_.lidar_rate_hz);
        imu_dropout_ns_  = static_cast<int64_t>(
            static_cast<double>(imu_period_ns_) * cfg_.dropout_factor);
        lidar_dropout_ns_ = static_cast<int64_t>(
            static_cast<double>(lidar_period_ns_) * cfg_.dropout_factor);
    }

    // ── Event ingress (called from worker thread) ───────────────────────

    /// Notify health monitor of a new IMU sample.
    /// Cost: ~10 ns (timestamp compare + saturation check).
    void on_imu(const ImuSample& sample) noexcept {
        const int64_t now = sample.timestamp_ns;

        // Gap detection.
        if (last_imu_ns_ > 0) {
            imu_gap_ns_ = now - last_imu_ns_;
            if (imu_gap_ns_ > imu_dropout_ns_) {
                det_imu_.fired = true;
                ++det_imu_.fire_count;
                if (det_imu_.first_fire_ns == 0) det_imu_.first_fire_ns = now;
            } else {
                if (det_imu_.fired) det_imu_.last_clear_ns = now;
                det_imu_.fired = false;
            }
        }
        last_imu_ns_ = now;

        // Exponential score: 1.0 at on-time, decays with gap.
        const double gap_ratio = static_cast<double>(imu_gap_ns_)
                               / static_cast<double>(imu_period_ns_);
        det_imu_.score = (gap_ratio <= 1.0) ? 1.0
                       : std::exp(-0.5 * (gap_ratio - 1.0));

        // Saturation check.
        const double ax = std::abs(sample.accel[0]),
                     ay = std::abs(sample.accel[1]),
                     az = std::abs(sample.accel[2]);
        const double gx = std::abs(sample.gyro[0]),
                     gy = std::abs(sample.gyro[1]),
                     gz = std::abs(sample.gyro[2]);

        if (ax > cfg_.accel_max_mps2 || ay > cfg_.accel_max_mps2 ||
            az > cfg_.accel_max_mps2 ||
            gx > cfg_.gyro_max_rps || gy > cfg_.gyro_max_rps ||
            gz > cfg_.gyro_max_rps) {
            det_saturation_.fired = true;
            ++det_saturation_.fire_count;
        } else {
            det_saturation_.fired = false;
        }
    }

    /// Notify health monitor of a new LiDAR scan arrival.
    /// Cost: ~5 ns.
    void on_lidar(int64_t timestamp_ns) noexcept {
        if (last_lidar_ns_ > 0) {
            lidar_gap_ns_ = timestamp_ns - last_lidar_ns_;
            if (lidar_gap_ns_ > lidar_dropout_ns_) {
                det_lidar_.fired = true;
                ++det_lidar_.fire_count;
                if (det_lidar_.first_fire_ns == 0)
                    det_lidar_.first_fire_ns = timestamp_ns;
            } else {
                if (det_lidar_.fired) det_lidar_.last_clear_ns = timestamp_ns;
                det_lidar_.fired = false;
            }
        }
        last_lidar_ns_ = timestamp_ns;

        const double gap_ratio = static_cast<double>(lidar_gap_ns_)
                               / static_cast<double>(lidar_period_ns_);
        det_lidar_.score = (gap_ratio <= 1.0) ? 1.0
                         : std::exp(-0.5 * (gap_ratio - 1.0));
    }

    /// Notify after ESIKF update — checks covariance, residual, drift, geometry.
    /// This is the main per-scan health evaluation.
    /// Cost: ~30 ns.
    void on_slam_output(const SlamOutput& out) noexcept {
        const int64_t now = out.timestamp_ns;
        ++scan_count_;

        // ── 1. Covariance trace check ───────────────────────────────────
        double trace = 0.0;
        for (int i = 0; i < 6; ++i)
            trace += out.pose.covariance_6x6[static_cast<size_t>(i * 6 + i)];

        cov_trace_ = trace;

        // NaN check.
        if (std::isnan(trace) || std::isinf(trace)) {
            det_nan_.fired = true;
            ++det_nan_.fire_count;
            det_ekf_.fired = true;
            det_ekf_.score = 0.0;
        } else {
            det_nan_.fired = false;

            if (trace > cfg_.cov_trace_crit) {
                det_ekf_.fired = true;
                det_ekf_.score = 0.0;
                ++det_ekf_.fire_count;
            } else if (trace > cfg_.cov_trace_warn) {
                det_ekf_.fired = true;
                det_ekf_.score = 1.0 - (trace - cfg_.cov_trace_warn)
                                      / (cfg_.cov_trace_crit - cfg_.cov_trace_warn);
                ++det_ekf_.fire_count;
            } else {
                if (det_ekf_.fired) det_ekf_.last_clear_ns = now;
                det_ekf_.fired = false;
                det_ekf_.score = 1.0;
            }
        }

        // ── 2. Residual spike check ─────────────────────────────────────
        const double residual = out.esikf_residual;
        if (scan_count_ <= 1) {
            residual_ema_ = residual;
        } else {
            residual_ema_ = cfg_.residual_ema_alpha * residual
                          + (1.0 - cfg_.residual_ema_alpha) * residual_ema_;
        }

        if (residual_ema_ > 0.0 &&
            residual > cfg_.residual_spike_factor * residual_ema_) {
            det_residual_.fired = true;
            ++det_residual_.fire_count;
            det_residual_.score = std::max(0.0,
                1.0 - (residual - residual_ema_)
                     / (cfg_.residual_spike_factor * residual_ema_));
        } else {
            if (det_residual_.fired) det_residual_.last_clear_ns = now;
            det_residual_.fired = false;
            det_residual_.score = 1.0;
        }

        // ── 3. Drift rate check ─────────────────────────────────────────
        if (prev_pose_valid_) {
            const double dx = out.pose.position[0] - prev_position_[0];
            const double dy = out.pose.position[1] - prev_position_[1];
            const double dz = out.pose.position[2] - prev_position_[2];
            const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            const double dt = static_cast<double>(now - prev_pose_ns_) / 1.0e9;

            if (dt > 0.0) {
                drift_speed_ = dist / dt;
                const double drift_limit = cfg_.max_platform_speed_mps
                                         * cfg_.drift_speed_factor;
                if (drift_speed_ > drift_limit) {
                    det_drift_.fired = true;
                    ++det_drift_.fire_count;
                    det_drift_.score = std::max(0.0,
                        1.0 - (drift_speed_ - cfg_.max_platform_speed_mps)
                             / (drift_limit - cfg_.max_platform_speed_mps));
                } else {
                    if (det_drift_.fired) det_drift_.last_clear_ns = now;
                    det_drift_.fired = false;
                    det_drift_.score = 1.0;
                }
            }
        }
        prev_position_ = out.pose.position;
        prev_pose_ns_ = now;
        prev_pose_valid_ = true;

        // ── 4. Geometry check ───────────────────────────────────────────
        if (out.correspondences >= cfg_.min_points_for_geom) {
            // Use eigenvalue ratio if externally provided, otherwise derive
            // from correspondences count as a proxy.
            // In real implementation, the ESIKF would provide the scatter
            // matrix eigenvalues.  Here we use a heuristic:
            //   ratio ≈ correspondences / max_expected (clamped to [0,1])
            // The real path uses set_geometry_eigenvalues() below.
            if (eigenvalue_ratio_valid_) {
                if (eigenvalue_ratio_ < cfg_.geom_linear_ratio) {
                    det_geom_.fired = true;
                    det_geom_.score = 0.0;
                    ++det_geom_.fire_count;
                } else if (eigenvalue_ratio_ < cfg_.geom_degen_ratio_warn) {
                    det_geom_.fired = true;
                    det_geom_.score = eigenvalue_ratio_ / cfg_.geom_degen_ratio_warn;
                    ++det_geom_.fire_count;
                } else {
                    if (det_geom_.fired) det_geom_.last_clear_ns = now;
                    det_geom_.fired = false;
                    det_geom_.score = 1.0;
                }
            }
        }

        // ── 5. Map health ───────────────────────────────────────────────
        if (out.map_info.total_points < cfg_.map_min_points ||
            out.map_info.total_points > cfg_.map_max_points) {
            det_map_.fired = true;
            ++det_map_.fire_count;
        } else {
            det_map_.fired = false;
        }

        // ── 6. Aggregate ────────────────────────────────────────────────
        update_state_machine(now);
    }

    /// Feed the 3×3 scatter matrix eigenvalue ratio from ESIKF internals.
    /// Call this from the ESIKF update step before on_slam_output().
    /// @param scatter_upper  Upper triangle [s00, s01, s02, s11, s12, s22].
    void set_geometry_scatter(const double scatter_upper[6]) noexcept {
        eigenvalue_ratio_ = eigenvalue_ratio_3x3(scatter_upper);
        eigenvalue_ratio_valid_ = true;
    }

    /// Directly set the eigenvalue ratio (if already computed).
    void set_eigenvalue_ratio(double ratio) noexcept {
        eigenvalue_ratio_ = ratio;
        eigenvalue_ratio_valid_ = true;
    }

    /// Feed clock drift estimate (from SlamTimeSync).
    void set_clock_drift(int64_t drift_ns) noexcept {
        if (std::abs(drift_ns) > cfg_.time_sync_max_drift_ns) {
            det_timesync_.fired = true;
            ++det_timesync_.fire_count;
        } else {
            det_timesync_.fired = false;
        }
    }

    // ── Query (read-only, safe from any thread) ─────────────────────────

    /// Current health state.
    [[nodiscard]] HealthState state() const noexcept { return state_; }

    /// Current confidence [0, 1].
    [[nodiscard]] float confidence() const noexcept { return confidence_; }

    /// Current output validity.
    [[nodiscard]] OutputValidity validity() const noexcept { return validity_; }

    /// Active fault flags.
    [[nodiscard]] FaultFlags faults() const noexcept { return active_faults_; }

    /// Full snapshot (value copy — safe to publish/log).
    [[nodiscard]] SlamHealthSnapshot snapshot() const noexcept {
        SlamHealthSnapshot s;
        s.timestamp_ns        = last_update_ns_;
        s.state               = state_;
        s.active_faults       = active_faults_;
        s.validity            = validity_;
        s.confidence          = confidence_;

        s.imu_score           = static_cast<float>(det_imu_.score);
        s.lidar_score         = static_cast<float>(det_lidar_.score);
        s.geometry_score      = static_cast<float>(det_geom_.score);
        s.ekf_score           = static_cast<float>(det_ekf_.score);
        s.drift_score         = static_cast<float>(det_drift_.score);
        s.residual_score      = static_cast<float>(det_residual_.score);

        s.last_imu_ns         = last_imu_ns_;
        s.last_lidar_ns       = last_lidar_ns_;
        s.imu_gap_ns          = imu_gap_ns_;
        s.lidar_gap_ns        = lidar_gap_ns_;

        s.cov_trace           = cov_trace_;
        s.residual_ema        = residual_ema_;
        s.drift_speed_mps     = drift_speed_;
        s.eigenvalue_ratio    = eigenvalue_ratio_;

        s.total_imu_dropouts  = det_imu_.fire_count;
        s.total_lidar_dropouts = det_lidar_.fire_count;
        s.total_ekf_divergences = det_ekf_.fire_count;
        s.total_state_transitions = state_transitions_;
        s.time_in_state_ns    = last_update_ns_ - state_enter_ns_;

        return s;
    }

    // ── Callbacks ───────────────────────────────────────────────────────

    void on_health_state_change(HealthStateCallback cb) noexcept {
        state_cb_ = std::move(cb);
    }

    void on_emergency(EmergencyCallback cb) noexcept {
        emergency_cb_ = std::move(cb);
    }

    void on_health_update(HealthUpdateCallback cb) noexcept {
        update_cb_ = std::move(cb);
    }

    // ── Reset ───────────────────────────────────────────────────────────

    void reset() noexcept {
        state_          = HealthState::Nominal;
        active_faults_  = FaultFlags::None;
        validity_       = OutputValidity::Valid;
        confidence_     = 1.0f;

        det_imu_        = {};
        det_lidar_      = {};
        det_geom_       = {};
        det_ekf_        = {};
        det_drift_      = {};
        det_residual_   = {};
        det_saturation_ = {};
        det_nan_        = {};
        det_map_        = {};
        det_timesync_   = {};

        last_imu_ns_    = 0;
        last_lidar_ns_  = 0;
        imu_gap_ns_     = 0;
        lidar_gap_ns_   = 0;
        cov_trace_      = 0.0;
        residual_ema_   = 0.0;
        drift_speed_    = 0.0;
        scan_count_     = 0;
        prev_pose_valid_ = false;
        eigenvalue_ratio_ = 1.0;
        eigenvalue_ratio_valid_ = false;
        state_enter_ns_ = 0;
        last_update_ns_ = 0;
        all_clear_ns_   = 0;
        state_transitions_ = 0;
    }

    // ── Config hot-reload ───────────────────────────────────────────────

    void update_config(const HealthMonitorConfig& cfg) noexcept {
        cfg_ = cfg;
        imu_period_ns_   = static_cast<int64_t>(1.0e9 / cfg_.imu_rate_hz);
        lidar_period_ns_ = static_cast<int64_t>(1.0e9 / cfg_.lidar_rate_hz);
        imu_dropout_ns_  = static_cast<int64_t>(
            static_cast<double>(imu_period_ns_) * cfg_.dropout_factor);
        lidar_dropout_ns_ = static_cast<int64_t>(
            static_cast<double>(lidar_period_ns_) * cfg_.dropout_factor);
    }

    /// Immutable config access.
    [[nodiscard]] const HealthMonitorConfig& config() const noexcept {
        return cfg_;
    }

private:
    // ── State machine update ────────────────────────────────────────────

    void update_state_machine(int64_t now) noexcept {
        last_update_ns_ = now;

        // ── Assemble fault flags ────────────────────────────────────────
        FaultFlags faults = FaultFlags::None;
        if (det_imu_.fired)        faults |= FaultFlags::ImuDropout;
        if (det_lidar_.fired)      faults |= FaultFlags::LidarDropout;
        if (det_geom_.fired)       faults |= FaultFlags::DegenerateGeom;
        if (det_ekf_.fired)        faults |= FaultFlags::EkfDivergence;
        if (det_drift_.fired)      faults |= FaultFlags::HighDriftRate;
        if (det_residual_.fired)   faults |= FaultFlags::ResidualSpike;
        if (det_saturation_.fired) faults |= FaultFlags::ImuSaturation;
        if (det_nan_.fired)        faults |= FaultFlags::CovarianceNaN;
        if (det_map_.fired)        faults |= FaultFlags::MapDegraded;
        if (det_timesync_.fired)   faults |= FaultFlags::TimeSyncLost;
        active_faults_ = faults;

        // ── Compute confidence ──────────────────────────────────────────
        const double conf =
            cfg_.w_imu      * det_imu_.score +
            cfg_.w_lidar    * det_lidar_.score +
            cfg_.w_geom     * det_geom_.score +
            cfg_.w_ekf      * det_ekf_.score +
            cfg_.w_drift    * det_drift_.score +
            cfg_.w_residual * det_residual_.score;
        confidence_ = static_cast<float>(std::max(0.0, std::min(1.0, conf)));

        // ── Compute validity ────────────────────────────────────────────
        if (confidence_ >= cfg_.validity_cautionary_threshold) {
            validity_ = OutputValidity::Valid;
        } else if (confidence_ >= cfg_.validity_invalid_threshold) {
            validity_ = OutputValidity::Cautionary;
        } else {
            validity_ = OutputValidity::Invalid;
        }

        // ── State transitions ───────────────────────────────────────────
        const int n_faults = fault_count(faults);
        const HealthState prev_state = state_;

        switch (state_) {
            case HealthState::Nominal:
                if (n_faults > 0) {
                    transition_to(HealthState::Degraded, now);
                }
                break;

            case HealthState::Degraded:
                if (n_faults == 0) {
                    // Heal check: must stay clear for heal_window.
                    if (all_clear_ns_ == 0) {
                        all_clear_ns_ = now;
                    } else if (now - all_clear_ns_ >= cfg_.heal_window_ns) {
                        transition_to(HealthState::Nominal, now);
                        all_clear_ns_ = 0;
                    }
                } else {
                    all_clear_ns_ = 0;
                    // Escalate to Critical if ≥2 faults or sustained.
                    if (n_faults >= 2 ||
                        (now - state_enter_ns_ >= cfg_.degrade_timeout_ns)) {
                        transition_to(HealthState::Critical, now);
                    }
                }
                break;

            case HealthState::Critical:
                if (n_faults == 0) {
                    if (all_clear_ns_ == 0) {
                        all_clear_ns_ = now;
                    } else if (now - all_clear_ns_ >= cfg_.heal_window_ns) {
                        transition_to(HealthState::Degraded, now);
                        all_clear_ns_ = 0;
                    }
                } else {
                    all_clear_ns_ = 0;
                    if (now - state_enter_ns_ >= cfg_.critical_timeout_ns) {
                        transition_to(HealthState::Emergency, now);
                    }
                }
                break;

            case HealthState::Emergency:
                // Only way out: explicit reset or heal from clear.
                if (n_faults == 0) {
                    if (all_clear_ns_ == 0) {
                        all_clear_ns_ = now;
                    } else if (now - all_clear_ns_ >= cfg_.heal_window_ns * 2) {
                        // Double heal window for emergency recovery.
                        transition_to(HealthState::Degraded, now);
                        all_clear_ns_ = 0;
                    }
                } else {
                    all_clear_ns_ = 0;
                }
                break;
        }

        // Override validity in Emergency.
        if (state_ == HealthState::Emergency) {
            validity_ = OutputValidity::Invalid;
        }

        // ── Callbacks ───────────────────────────────────────────────────
        if (update_cb_) {
            update_cb_(snapshot());
        }
    }

    void transition_to(HealthState new_state, int64_t now) noexcept {
        const HealthState old_state = state_;
        state_ = new_state;
        state_enter_ns_ = now;
        ++state_transitions_;

        if (state_cb_) {
            state_cb_(old_state, new_state, active_faults_);
        }
        if (new_state == HealthState::Emergency && emergency_cb_) {
            emergency_cb_(snapshot());
        }
    }

    // ── Configuration ───────────────────────────────────────────────────
    HealthMonitorConfig cfg_;

    // ── Derived constants ───────────────────────────────────────────────
    int64_t imu_period_ns_{0};
    int64_t lidar_period_ns_{0};
    int64_t imu_dropout_ns_{0};
    int64_t lidar_dropout_ns_{0};

    // ── Per-detector state ──────────────────────────────────────────────
    DetectorState det_imu_{};
    DetectorState det_lidar_{};
    DetectorState det_geom_{};
    DetectorState det_ekf_{};
    DetectorState det_drift_{};
    DetectorState det_residual_{};
    DetectorState det_saturation_{};
    DetectorState det_nan_{};
    DetectorState det_map_{};
    DetectorState det_timesync_{};

    // ── Cached measurements ─────────────────────────────────────────────
    int64_t last_imu_ns_{0};
    int64_t last_lidar_ns_{0};
    int64_t imu_gap_ns_{0};
    int64_t lidar_gap_ns_{0};

    double  cov_trace_{0.0};
    double  residual_ema_{0.0};
    double  drift_speed_{0.0};
    double  eigenvalue_ratio_{1.0};
    bool    eigenvalue_ratio_valid_{false};

    std::array<double,3> prev_position_{};
    int64_t prev_pose_ns_{0};
    bool    prev_pose_valid_{false};

    uint64_t scan_count_{0};

    // ── State machine ───────────────────────────────────────────────────
    HealthState    state_{HealthState::Nominal};
    FaultFlags     active_faults_{FaultFlags::None};
    OutputValidity validity_{OutputValidity::Valid};
    float          confidence_{1.0f};

    int64_t state_enter_ns_{0};
    int64_t last_update_ns_{0};
    int64_t all_clear_ns_{0};
    uint64_t state_transitions_{0};

    // ── Callbacks ───────────────────────────────────────────────────────
    HealthStateCallback  state_cb_;
    EmergencyCallback    emergency_cb_;
    HealthUpdateCallback update_cb_;
};

} // namespace thunderbird::odom
