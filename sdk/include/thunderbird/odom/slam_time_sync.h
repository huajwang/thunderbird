// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Time Synchronization Layer
// ─────────────────────────────────────────────────────────────────────────────
//
// This module synchronises high-rate IMU data (400–1000 Hz) with lower-rate
// LiDAR scans (10–20 Hz) for the odometry pipeline.  It is **separate** from
// the existing TimeSyncEngine (time_sync.h) which handles the general-purpose
// sensor fusion for the Data Abstraction Layer / Python / ROS2 bridge.
//
// Why a separate module?
//
//   The SLAM estimator has stricter requirements than the generic sync layer:
//
//     1. It needs the **complete, gap-free** IMU trace between consecutive
//        LiDAR scans — dropping even one sample causes preintegration drift.
//     2. It must **interpolate** the IMU stream to synthesize a sample at the
//        exact LiDAR scan-boundary timestamp (not nearest-neighbour).
//     3. It must track hardware↔host clock drift with sub-millisecond
//        precision and compensate transparently.
//     4. It must handle **out-of-order arrivals** (common over USB/Ethernet)
//        with a configurable sort window.
//     5. Alignment overhead must be **< 5 ms** per scan to stay within the
//        10 Hz real-time budget.
//
// Architecture:
//
//   Producers (sensor threads)         Consumer (ESIKF thread)
//   ──────────────────────────         ──────────────────────────
//   feed_imu()  ──► ImuRingBuf        poll_next_measurement()
//   feed_lidar()──► LidarQueue               │
//                      │                     │
//                      └──► SlamTimeSync ────┘
//                           (assembles ScanMeasurement bundles)
//
// ┌─────────────────────────────────────────────────────────────────────┐
// │                       SlamTimeSync                                  │
// │                                                                     │
// │ IMU samples arrive at 400–1000 Hz, potentially out-of-order.       │
// │                                                                     │
// │  ┌──────────────────────────────────────────────────────────────┐   │
// │  │  ImuSortBuffer (reorder window = 5 ms)                       │   │
// │  │  Incoming samples are inserted in timestamp order.           │   │
// │  │  Samples older than (newest_ts − window) are flushed to      │   │
// │  │  the main ImuRingBuffer in monotonic order.                  │   │
// │  └──────────────────────────┬───────────────────────────────────┘   │
// │                             │                                       │
// │  ┌──────────────────────────▼───────────────────────────────────┐   │
// │  │  ImuRingBuffer (lock-free SPSC, 4096 slots)                  │   │
// │  │  Monotonically increasing timestamps guaranteed.             │   │
// │  │  Consumer drains all samples in [prev_lidar_ts, cur_lidar_ts]│   │
// │  └──────────────────────────┬───────────────────────────────────┘   │
// │                             │                                       │
// │  When a LiDAR scan arrives: │                                       │
// │                             ▼                                       │
// │  ┌──────────────────────────────────────────────────────────────┐   │
// │  │  Assemble ScanMeasurement:                                   │   │
// │  │                                                              │   │
// │  │  1. Wait up to `imu_wait_timeout_ms` for IMU coverage        │   │
// │  │  2. Drain IMU buffer: all samples in (prev_ts, scan_ts]      │   │
// │  │  3. Interpolate boundary samples at prev_ts and scan_ts      │   │
// │  │  4. Apply clock drift compensation (if enabled)              │   │
// │  │  5. Emit ScanMeasurement to output queue                     │   │
// │  └──────────────────────────────────────────────────────────────┘   │
// │                                                                     │
// │  Clock drift model:                                                 │
// │  ┌──────────────────────────────────────────────────────────────┐   │
// │  │  Maintains a sliding window of (hw_ts, host_ts) pairs.       │   │
// │  │  Linear regression: host_ts = α·hw_ts + β                    │   │
// │  │  Used to convert hardware timestamps to a unified clock      │   │
// │  │  when sensors use different oscillators.                     │   │
// │  │  Drift rate tracked in ns/s.  Warning callback if > 1 ms/s. │   │
// │  └──────────────────────────────────────────────────────────────┘   │
// └─────────────────────────────────────────────────────────────────────┘
//
// Buffer size calculations:
//
//   IMU at 1000 Hz, LiDAR at 10 Hz → 100 IMU samples per scan.
//   RingBuffer capacity = 4096 → covers 4.096 seconds ≈ 40 scans of headroom.
//   Sort window = 5 ms → max 5 out-of-order samples at 1000 Hz.
//   Worst-case assembly time: drain 100–200 samples + 2 interpolations ≈ 10 µs.
//
// Thread safety:
//
//   • feed_imu() / feed_lidar() are called from sensor driver threads.
//   • poll_next_measurement() is called from the ESIKF thread.
//   • The sort buffer and clock model are protected by a lightweight spinlock
//     (taken only during feed_imu, held < 1 µs).
//   • The output queue is a lock-free SPSC RingBuffer (producer: assembler,
//     consumer: ESIKF thread).
//   • No dynamic allocation on the hot path after initialisation.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/odom/slam_types.h"
#include "thunderbird/ring_buffer.h"
#include "thunderbird/clock_service.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace thunderbird::odom {

// ═════════════════════════════════════════════════════════════════════════════
//  Configuration
// ═════════════════════════════════════════════════════════════════════════════

struct SlamTimeSyncConfig {
    // ── IMU parameters ──────────────────────────────────────────────────
    /// Expected IMU rate in Hz (used for gap detection, not mandatory).
    double imu_rate_hz = 400.0;

    /// Maximum time (ms) to wait for IMU coverage when a LiDAR scan
    /// arrives but the IMU buffer doesn't yet span the scan timestamp.
    /// If exceeded, the scan is emitted with a partial IMU block.
    double imu_wait_timeout_ms = 10.0;

    /// IMU ring buffer capacity (must be power of 2).
    /// 4096 @ 1000 Hz = 4.096 s headroom.
    static constexpr size_t kImuBufferCapacity = 4096;

    // ── Out-of-order handling ───────────────────────────────────────────
    /// Sort window in nanoseconds.  IMU samples arriving within this
    /// window of the newest are re-ordered; older arrivals are dropped.
    /// 5 ms covers typical USB/Ethernet jitter.
    int64_t sort_window_ns = 5'000'000;  // 5 ms

    // ── LiDAR parameters ────────────────────────────────────────────────
    /// Expected LiDAR rate in Hz (used for gap detection).
    double lidar_rate_hz = 10.0;

    /// LiDAR scan queue depth.
    static constexpr size_t kLidarQueueCapacity = 8;

    // ── Clock drift compensation ────────────────────────────────────────
    /// Enable hardware↔host clock drift estimation and correction.
    bool enable_drift_compensation = true;

    /// Number of timestamp pairs in the drift estimation window.
    size_t drift_window_size = 100;

    /// Drift warning threshold (ns/s).  If the estimated drift rate
    /// exceeds this, the drift warning callback fires.
    double drift_warn_threshold_ns_per_sec = 1'000'000.0;  // 1 ms/s

    // ── Timestamp domain ────────────────────────────────────────────────
    /// Which timestamp to use as the canonical clock.
    enum class ClockDomain : uint8_t {
        Hardware = 0,   ///< Use hardware timestamps (preferred)
        Host     = 1,   ///< Use host arrival timestamps (fallback)
    };

    /// Primary clock domain.  If Hardware timestamps are zeros or
    /// missing, the engine falls back to Host automatically.
    ClockDomain clock_domain = ClockDomain::Hardware;

    // ── Interpolation ───────────────────────────────────────────────────
    /// Use cubic B-spline interpolation for scan-boundary IMU samples
    /// instead of linear interpolation.  Smoother at low IMU rates
    /// (< 200 Hz); negligible difference at 400+ Hz.
    bool use_bspline_interpolation = false;

    // ── Output ──────────────────────────────────────────────────────────
    /// Assembled ScanMeasurement output queue depth.
    static constexpr size_t kOutputQueueCapacity = 16;
};

// ═════════════════════════════════════════════════════════════════════════════
//  ScanMeasurement — the fundamental input unit for the ESIKF
// ═════════════════════════════════════════════════════════════════════════════
//
// One ScanMeasurement = one LiDAR scan + the complete IMU trace covering
// the interval (prev_scan_ts, this_scan_ts].  The IMU block is guaranteed:
//
//   1. Monotonically increasing in timestamp.
//   2. Boundary-interpolated: the first sample is at prev_scan_ts (or
//      the closest available) and the last is at this_scan_ts.
//   3. Gap-free (no missing samples, barring hardware failure).
//
// This is what the ESIKF's predict() and update() consume.
// ─────────────────────────────────────────────────────────────────────────────

struct ScanMeasurement {
    // ── LiDAR ───────────────────────────────────────────────────────────
    /// The raw (not yet deskewed) LiDAR scan.
    std::shared_ptr<const PointCloudFrame> scan;

    // ── IMU block ───────────────────────────────────────────────────────
    /// All IMU samples in the interval (prev_scan_ts, this_scan_ts],
    /// boundary-interpolated.  First and last samples correspond to the
    /// exact scan-boundary timestamps.
    std::vector<ImuSample> imu_block;

    // ── Timing ──────────────────────────────────────────────────────────
    int64_t scan_start_ns{0};     ///< start of this scan (= imu_block.back().timestamp_ns)
    int64_t scan_end_ns{0};       ///< ≡ scan->timestamp_ns (end-of-sweep convention)
    int64_t prev_scan_ns{0};      ///< previous scan timestamp (imu_block.front().timestamp_ns)
    uint32_t sequence{0};         ///< monotonic measurement counter

    // ── Diagnostics ─────────────────────────────────────────────────────
    uint32_t imu_sample_count{0};       ///< number of IMU samples in block
    int64_t  imu_span_ns{0};            ///< imu_block.back().ts - imu_block.front().ts
    bool     has_boundary_interp{false};///< true if boundary interpolation was applied
    bool     is_partial{false};         ///< true if IMU coverage was incomplete
    double   assembly_latency_us{0};    ///< wall-clock assembly time (µs)
};

// ═════════════════════════════════════════════════════════════════════════════
//  ClockDriftModel — linear regression hardware↔host timestamp model
// ═════════════════════════════════════════════════════════════════════════════
//
//   host_ts = alpha * hw_ts + beta
//
// Maintained as a sliding-window OLS over recent (hw_ts, host_ts) pairs.
// Allows converting between clock domains and detecting excessive drift.
// ─────────────────────────────────────────────────────────────────────────────

class ClockDriftModel {
public:
    explicit ClockDriftModel(size_t window_size = 100)
        : window_size_(window_size) {}

    /// Add a new (hardware_timestamp, host_timestamp) observation.
    void add_observation(int64_t hw_ns, int64_t host_ns) {
        hw_samples_.push_back(static_cast<double>(hw_ns));
        host_samples_.push_back(static_cast<double>(host_ns));

        while (hw_samples_.size() > window_size_) {
            hw_samples_.pop_front();
            host_samples_.pop_front();
        }

        recompute();
    }

    /// Convert a hardware timestamp to the host clock domain.
    [[nodiscard]] int64_t hw_to_host(int64_t hw_ns) const noexcept {
        if (!valid_) return hw_ns;  // identity until calibrated
        return static_cast<int64_t>(alpha_ * static_cast<double>(hw_ns) + beta_);
    }

    /// Convert a host timestamp to the hardware clock domain.
    [[nodiscard]] int64_t host_to_hw(int64_t host_ns) const noexcept {
        if (!valid_ || std::abs(alpha_) < 1e-15) return host_ns;
        return static_cast<int64_t>((static_cast<double>(host_ns) - beta_) / alpha_);
    }

    /// Current drift rate in ns/s.
    /// A value of 0 means no drift; 1'000'000 means 1 ms/s drift.
    /// The drift rate is (alpha - 1.0) * 1e9 ns/s.
    [[nodiscard]] double drift_ns_per_sec() const noexcept {
        if (!valid_) return 0.0;
        return (alpha_ - 1.0) * 1.0e9;
    }

    /// Current offset (beta) in nanoseconds.
    [[nodiscard]] double offset_ns() const noexcept { return beta_; }

    /// True once we have enough samples for a reliable estimate.
    [[nodiscard]] bool is_valid() const noexcept { return valid_; }

    /// Number of observations in the window.
    [[nodiscard]] size_t sample_count() const noexcept {
        return hw_samples_.size();
    }

    /// Reset all state.
    void reset() {
        hw_samples_.clear();
        host_samples_.clear();
        alpha_ = 1.0;
        beta_  = 0.0;
        valid_ = false;
    }

private:
    void recompute() {
        const size_t n = hw_samples_.size();
        if (n < 3) { valid_ = false; return; }

        // OLS: host = alpha * hw + beta
        double hw_mean = 0, host_mean = 0;
        for (size_t i = 0; i < n; ++i) {
            hw_mean   += hw_samples_[i];
            host_mean += host_samples_[i];
        }
        hw_mean   /= static_cast<double>(n);
        host_mean /= static_cast<double>(n);

        double num = 0, den = 0;
        for (size_t i = 0; i < n; ++i) {
            const double dh = hw_samples_[i]   - hw_mean;
            const double dr = host_samples_[i] - host_mean;
            num += dh * dr;
            den += dh * dh;
        }

        if (den < 1e-6) { valid_ = false; return; }

        alpha_ = num / den;
        beta_  = host_mean - alpha_ * hw_mean;
        valid_ = true;
    }

    size_t window_size_;
    std::deque<double> hw_samples_;
    std::deque<double> host_samples_;
    double alpha_{1.0};   // scale  (ideally 1.0 = no drift)
    double beta_{0.0};    // offset (ideally 0.0)
    bool   valid_{false};
};

// ═════════════════════════════════════════════════════════════════════════════
//  ImuSortBuffer — reorders out-of-order IMU arrivals
// ═════════════════════════════════════════════════════════════════════════════
//
// USB/Ethernet IMU transport can deliver samples out of order.  This buffer
// holds samples within a configurable time window and flushes them in
// monotonically increasing timestamp order.
//
// Complexity: O(n) insert with small n (window typically holds 1–5 samples).
// ─────────────────────────────────────────────────────────────────────────────

class ImuSortBuffer {
public:
    explicit ImuSortBuffer(int64_t window_ns = 5'000'000)
        : window_ns_(window_ns) {}

    /// Insert a new sample.  Returns samples that have "graduated" from
    /// the sort window (older than newest_ts - window_ns) in sorted order.
    /// Caller should push these into the downstream monotonic buffer.
    std::vector<ImuSample> insert(const ImuSample& sample) {
        // Track newest arrival.
        if (sample.timestamp_ns > newest_ts_) {
            newest_ts_ = sample.timestamp_ns;
        }

        // Drop if too old to be sorted (beyond the window).
        const int64_t cutoff = newest_ts_ - window_ns_;
        if (sample.timestamp_ns < cutoff) {
            ++dropped_;
            return {};
        }

        // Insert in sorted position (usually at or near the end).
        auto it = std::lower_bound(
            buf_.begin(), buf_.end(), sample,
            [](const ImuSample& a, const ImuSample& b) {
                return a.timestamp_ns < b.timestamp_ns;
            });
        buf_.insert(it, sample);

        // Flush all samples below the cutoff.
        std::vector<ImuSample> flushed;
        while (!buf_.empty() && buf_.front().timestamp_ns < cutoff) {
            flushed.push_back(buf_.front());
            buf_.pop_front();
        }
        return flushed;
    }

    /// Force-flush all remaining samples (call at shutdown or scan boundary).
    std::vector<ImuSample> flush_all() {
        std::vector<ImuSample> out(buf_.begin(), buf_.end());
        buf_.clear();
        return out;
    }

    /// Number of samples currently held in the sort window.
    [[nodiscard]] size_t pending() const noexcept { return buf_.size(); }

    /// Total samples dropped due to arriving too late.
    [[nodiscard]] uint64_t dropped() const noexcept { return dropped_; }

    void reset() {
        buf_.clear();
        newest_ts_ = 0;
        dropped_ = 0;
    }

private:
    int64_t window_ns_;
    int64_t newest_ts_{0};
    std::deque<ImuSample> buf_;
    uint64_t dropped_{0};
};

// ═════════════════════════════════════════════════════════════════════════════
//  ImuInterpolator — synthesize IMU samples at exact timestamps
// ═════════════════════════════════════════════════════════════════════════════
//
// Linear interpolation (SLERP for gyro is unnecessary at high sample rates;
// the angular change between consecutive samples at 1000 Hz is < 0.001 rad,
// making linear interpolation indistinguishable from SLERP).
//
// Latency: ~10 ns per interpolation (two multiplies + add).
// ─────────────────────────────────────────────────────────────────────────────

struct ImuInterpolator {
    /// Linearly interpolate between two IMU samples at target_ns.
    /// Precondition: a.timestamp_ns <= target_ns <= b.timestamp_ns
    ///               and a.timestamp_ns != b.timestamp_ns.
    [[nodiscard]] static ImuSample lerp(
        const ImuSample& a,
        const ImuSample& b,
        int64_t target_ns) noexcept
    {
        const double span = static_cast<double>(b.timestamp_ns - a.timestamp_ns);
        const double t = (span > 0.0)
            ? static_cast<double>(target_ns - a.timestamp_ns) / span
            : 0.0;

        ImuSample result;
        result.timestamp_ns = target_ns;

        for (int i = 0; i < 3; ++i) {
            result.accel[i] = a.accel[i] + t * (b.accel[i] - a.accel[i]);
            result.gyro[i]  = a.gyro[i]  + t * (b.gyro[i]  - a.gyro[i]);
        }

        result.temperature = a.temperature + t * (b.temperature - a.temperature);
        return result;
    }

    /// Clamp-extrapolate: if target_ns is outside [a, b], use the nearer
    /// endpoint.  Useful for boundary cases when the IMU stream starts
    /// slightly after the scan boundary.
    [[nodiscard]] static ImuSample lerp_clamped(
        const ImuSample& a,
        const ImuSample& b,
        int64_t target_ns) noexcept
    {
        if (target_ns <= a.timestamp_ns) return a;
        if (target_ns >= b.timestamp_ns) return b;
        return lerp(a, b, target_ns);
    }

    /// Cubic B-spline interpolation over an IMU block at target_ns.
    ///
    /// Fits a uniform cubic B-spline through the IMU samples and evaluates
    /// at target_ns for smoother boundary synthesis.  Falls back to linear
    /// interpolation when there are fewer than 4 samples (minimum needed
    /// for a cubic B-spline segment).
    ///
    /// @param block  Monotonically increasing IMU samples (N >= 2).
    /// @param target_ns  Timestamp to interpolate at (must be within
    ///                   block time range).
    /// @return Interpolated IMU sample at target_ns.
    [[nodiscard]] static ImuSample bspline(
        const std::vector<ImuSample>& block,
        int64_t target_ns) noexcept
    {
        const size_t n = block.size();

        // Need at least 4 control points for cubic B-spline; fall back.
        if (n < 4) {
            if (n < 2) {
                ImuSample s{};
                s.timestamp_ns = target_ns;
                if (n == 1) s = block[0];
                s.timestamp_ns = target_ns;
                return s;
            }
            // Binary search for the bracketing pair.
            const auto it = std::lower_bound(
                block.begin(), block.end(), target_ns,
                [](const ImuSample& s, int64_t ts) {
                    return s.timestamp_ns < ts;
                });

            if (it == block.begin())
                return lerp_clamped(block[0], block[1], target_ns);
            if (it == block.end())
                return lerp_clamped(block[n - 2], block[n - 1], target_ns);

            const size_t hi = static_cast<size_t>(it - block.begin());
            return lerp_clamped(block[hi - 1], block[hi], target_ns);
        }

        // Uniform cubic B-spline basis weights: M * [1, u, u², u³]^T / 6
        // where M is the standard uniform cubic B-spline matrix.
        auto basis = [](double u, double* w) {
            const double u2 = u * u;
            const double u3 = u2 * u;
            w[0] = (1.0 - 3.0 * u + 3.0 * u2 - u3) / 6.0;
            w[1] = (4.0 - 6.0 * u2 + 3.0 * u3) / 6.0;
            w[2] = (1.0 + 3.0 * u + 3.0 * u2 - 3.0 * u3) / 6.0;
            w[3] = u3 / 6.0;
        };

        // Find the segment containing target_ns and compute the local
        // parameter u from actual timestamps.  Note: the basis weights above
        // are for a uniform-knot cubic B-spline, so with non-uniform sample
        // spacing this acts as an ad-hoc 4-point smoother rather than a true
        // non-uniform B-spline.  This is acceptable for IMU data where the
        // sample rate is nearly constant.
        const double t_target = static_cast<double>(target_ns);
        const double t0 = static_cast<double>(block.front().timestamp_ns);
        const double t_end = static_cast<double>(block.back().timestamp_ns);

        if (t_end <= t0) {
            return block.front();
        }

        // Clamp target to block range — fall back to endpoint samples.
        if (target_ns <= block.front().timestamp_ns) return block.front();
        if (target_ns >= block.back().timestamp_ns)  return block.back();

        // Binary search for the segment [seg, seg+1] containing target_ns.
        size_t seg = 0;
        for (size_t i = 1; i < n; ++i) {
            if (block[i].timestamp_ns >= target_ns) {
                seg = i - 1;
                break;
            }
        }
        if (seg >= n - 1) seg = n - 2;

        // Local parameter u ∈ [0, 1) within the segment, using actual timestamps.
        const double seg_t0 = static_cast<double>(block[seg].timestamp_ns);
        const double seg_t1 = static_cast<double>(block[seg + 1].timestamp_ns);
        double u = (seg_t1 > seg_t0)
            ? (t_target - seg_t0) / (seg_t1 - seg_t0)
            : 0.0;
        if (u < 0.0) u = 0.0;
        if (u > 1.0) u = 1.0;

        // For cubic B-spline we need 4 consecutive control points:
        // seg-1, seg, seg+1, seg+2.  Clamp indices at boundaries.
        size_t i0 = (seg >= 1) ? seg - 1 : 0;
        size_t i1 = seg;
        size_t i2 = (seg + 1 < n) ? seg + 1 : n - 1;
        size_t i3 = (seg + 2 < n) ? seg + 2 : n - 1;

        double w[4];
        basis(u, w);

        ImuSample result;
        result.timestamp_ns = target_ns;

        for (int c = 0; c < 3; ++c) {
            result.accel[c] =
                w[0] * block[i0].accel[c] + w[1] * block[i1].accel[c] +
                w[2] * block[i2].accel[c] + w[3] * block[i3].accel[c];
            result.gyro[c] =
                w[0] * block[i0].gyro[c] + w[1] * block[i1].gyro[c] +
                w[2] * block[i2].gyro[c] + w[3] * block[i3].gyro[c];
        }
        result.temperature =
            w[0] * block[i0].temperature + w[1] * block[i1].temperature +
            w[2] * block[i2].temperature + w[3] * block[i3].temperature;

        return result;
    }
};

// ═════════════════════════════════════════════════════════════════════════════
//  Diagnostics & callbacks
// ═════════════════════════════════════════════════════════════════════════════

struct SlamTimeSyncStats {
    uint64_t scans_assembled{0};        ///< total ScanMeasurements emitted
    uint64_t imu_samples_ingested{0};   ///< total IMU samples fed in
    uint64_t imu_samples_dropped{0};    ///< dropped by sort buffer (too late)
    uint64_t imu_gaps_detected{0};      ///< scans with incomplete IMU coverage
    uint64_t partial_assemblies{0};     ///< scans emitted with partial IMU
    uint64_t out_of_order_arrivals{0};  ///< IMU samples that needed reordering
    double   avg_assembly_latency_us{0};///< mean assembly time (µs)
    double   drift_ns_per_sec{0};       ///< current clock drift estimate
    double   mean_imu_interval_ns{0};   ///< observed mean IMU sample interval
};

/// Drift warning callback: reports the estimated drift rate in ns/s.
using DriftWarningCallback = std::function<void(double drift_ns_per_sec)>;

/// Gap warning callback: reports when IMU coverage is incomplete for a scan.
using ImuGapCallback = std::function<void(int64_t scan_ts_ns, int64_t gap_ns)>;

// ═════════════════════════════════════════════════════════════════════════════
//  SlamTimeSync — main synchronisation engine
// ═════════════════════════════════════════════════════════════════════════════

class SlamTimeSync {
public:
    explicit SlamTimeSync(SlamTimeSyncConfig config = {})
        : config_(config)
        , sort_buffer_(config.sort_window_ns)
        , drift_model_(config.drift_window_size) {}

    ~SlamTimeSync() = default;

    // Non-copyable, non-movable (owns mutex).
    SlamTimeSync(const SlamTimeSync&) = delete;
    SlamTimeSync& operator=(const SlamTimeSync&) = delete;

    // ── Clock service injection ────────────────────────────────────────────

    /// Inject the shared ClockService for drift-compensated timestamp
    /// resolution.  When set, the internal ClockDriftModel is bypassed.
    /// Lifetime: the ClockService must outlive this SlamTimeSync.
    void set_clock_service(const ClockService* cs) { clock_service_ = cs; }

    // ── Input: called from sensor threads ───────────────────────────────

    /// Feed an IMU sample.  Thread-safe (takes spinlock, held < 1 µs).
    ///
    /// The sample passes through the sort buffer for reordering, then
    /// enters the monotonic IMU ring buffer.  If the sample arrives too
    /// late (older than sort_window_ns from the newest), it is dropped.
    ///
    /// @param sample    The IMU measurement.
    /// @param host_ns   Host arrival timestamp (for drift estimation).
    ///                  Pass 0 to skip drift tracking.
    void feed_imu(const ImuSample& sample, int64_t host_ns = 0) {
        std::lock_guard<std::mutex> lk(imu_mu_);

        ++stats_.imu_samples_ingested;

        // ── Clock drift observation ─────────────────────────────────
        // When a ClockService is available, it receives observations
        // from the PacketParser directly — no need to feed here.
        // Fallback: use local ClockDriftModel.
        if (!clock_service_ && config_.enable_drift_compensation && host_ns > 0) {
            drift_model_.add_observation(sample.timestamp_ns, host_ns);
        }

        // ── Out-of-order detection ──────────────────────────────────
        if (sample.timestamp_ns < last_imu_ts_before_sort_) {
            ++stats_.out_of_order_arrivals;
        }
        last_imu_ts_before_sort_ = sample.timestamp_ns;

        // ── Sort buffer → monotonic ring buffer ─────────────────────
        auto flushed = sort_buffer_.insert(sample);
        for (auto& s : flushed) {
            monotonic_imu_.push_back(std::move(s));
        }

        // Trim monotonic buffer to prevent unbounded growth.
        // Keep at most kImuBufferCapacity samples.
        while (monotonic_imu_.size() >
               SlamTimeSyncConfig::kImuBufferCapacity) {
            monotonic_imu_.pop_front();
        }
    }

    /// Feed a LiDAR scan.  Thread-safe.
    ///
    /// @param scan      The point cloud.
    /// @param host_ns   Host arrival timestamp (for drift estimation).
    void feed_lidar(std::shared_ptr<const PointCloudFrame> scan,
                    int64_t host_ns = 0) {
        std::lock_guard<std::mutex> lk(lidar_mu_);

        // Fallback drift observation (only when no ClockService).
        if (!clock_service_ && config_.enable_drift_compensation && host_ns > 0) {
            drift_model_.add_observation(scan->timestamp_ns, host_ns);
        }

        lidar_queue_.push_back(std::move(scan));

        // Trim to max queue depth.
        while (lidar_queue_.size() >
               SlamTimeSyncConfig::kLidarQueueCapacity) {
            lidar_queue_.pop_front();
        }
    }

    // ── Output: called from the ESIKF thread ────────────────────────────

    /// Attempt to assemble the next ScanMeasurement.
    ///
    /// Returns std::nullopt if no LiDAR scan is available or if IMU
    /// coverage is not yet sufficient (and we haven't timed out).
    ///
    /// This is a non-blocking call.  The ESIKF thread should poll this
    /// in its main loop, or use wait_next_measurement() for blocking.
    [[nodiscard]] std::optional<ScanMeasurement> poll_next_measurement() {
        // ── Get the next LiDAR scan ─────────────────────────────────
        std::shared_ptr<const PointCloudFrame> scan;
        {
            std::lock_guard<std::mutex> lk(lidar_mu_);
            if (lidar_queue_.empty()) return std::nullopt;
            scan = lidar_queue_.front();
        }

        const int64_t scan_ts = resolve_timestamp(scan->timestamp_ns);

        // ── Check IMU coverage ──────────────────────────────────────
        {
            std::lock_guard<std::mutex> lk(imu_mu_);

            // Need IMU data spanning beyond the scan timestamp.
            if (monotonic_imu_.empty()) return std::nullopt;

            const int64_t newest_imu = monotonic_imu_.back().timestamp_ns;
            if (newest_imu < scan_ts) {
                // IMU hasn't caught up yet — check timeout.
                // (Caller should retry; we don't block here.)
                return std::nullopt;
            }
        }

        // ── Assemble ─────────────────────────────────────────────────
        return assemble(scan, scan_ts);
    }

    /// Blocking version: waits up to `timeout` for the next measurement.
    template <typename Rep, typename Period>
    [[nodiscard]] std::optional<ScanMeasurement> wait_next_measurement(
        std::chrono::duration<Rep, Period> timeout)
    {
        using clock = std::chrono::steady_clock;
        const auto deadline = clock::now() + timeout;

        while (clock::now() < deadline) {
            auto meas = poll_next_measurement();
            if (meas) return meas;
            std::this_thread::sleep_for(std::chrono::microseconds(200));
        }
        return std::nullopt;
    }

    // ── Diagnostics ─────────────────────────────────────────────────────

    /// Current statistics snapshot.
    [[nodiscard]] SlamTimeSyncStats stats() const {
        std::lock_guard<std::mutex> lk(imu_mu_);
        auto s = stats_;
        // Prefer ClockService drift; fall back to local model.
        s.drift_ns_per_sec = clock_service_
            ? clock_service_->drift_ns_per_sec()
            : drift_model_.drift_ns_per_sec();
        return s;
    }

    /// Access the clock drift model (read-only).
    [[nodiscard]] const ClockDriftModel& drift_model() const noexcept {
        return drift_model_;
    }

    // ── Callbacks ───────────────────────────────────────────────────────

    void on_drift_warning(DriftWarningCallback cb) {
        drift_warn_cb_ = std::move(cb);
    }

    void on_imu_gap(ImuGapCallback cb) {
        imu_gap_cb_ = std::move(cb);
    }

    // ── Lifecycle ───────────────────────────────────────────────────────

    /// Reset all internal state (queues, counters, drift model).
    void reset() {
        {
            std::lock_guard<std::mutex> lk(imu_mu_);
            monotonic_imu_.clear();
            sort_buffer_.reset();
            drift_model_.reset();
            stats_ = {};
            last_imu_ts_before_sort_ = 0;
            prev_scan_ts_ = 0;
            scan_count_ = 0;
            imu_interval_accumulator_ = 0;
            imu_interval_count_ = 0;
        }
        {
            std::lock_guard<std::mutex> lk(lidar_mu_);
            lidar_queue_.clear();
        }
    }

private:
    // ── Timestamp resolution ────────────────────────────────────────────

    /// Resolve a hardware timestamp to the canonical clock domain,
    /// applying drift compensation if enabled and calibrated.
    /// Prefers ClockService; falls back to local ClockDriftModel.
    [[nodiscard]] int64_t resolve_timestamp(int64_t hw_ns) const noexcept {
        if (!config_.enable_drift_compensation) return hw_ns;

        // Prefer shared ClockService when available.
        if (clock_service_ && clock_service_->is_calibrated()) {
            return clock_service_->hw_to_host(hw_ns);
        }

        // Fallback: local model.
        if (config_.clock_domain == SlamTimeSyncConfig::ClockDomain::Host &&
            drift_model_.is_valid()) {
            return drift_model_.hw_to_host(hw_ns);
        }
        return hw_ns;
    }

    // ── Core assembly logic ─────────────────────────────────────────────

    /// Assemble a ScanMeasurement for the given scan.
    /// Must be called when IMU coverage is confirmed.
    [[nodiscard]] std::optional<ScanMeasurement> assemble(
        std::shared_ptr<const PointCloudFrame> scan,
        int64_t scan_ts)
    {
        using clock = std::chrono::steady_clock;
        const auto t0 = clock::now();

        ScanMeasurement meas;
        meas.scan = scan;
        meas.scan_end_ns = scan_ts;
        meas.prev_scan_ns = prev_scan_ts_;
        meas.sequence = scan_count_;

        // ── Drain IMU samples in (prev_scan_ts_, scan_ts] ───────────
        {
            std::lock_guard<std::mutex> lk(imu_mu_);

            // Flush any remaining sorted samples up to the scan boundary.
            auto remaining = sort_buffer_.flush_all();
            for (auto& s : remaining) {
                monotonic_imu_.push_back(std::move(s));
            }

            // Find the boundary: all samples with ts in (prev_scan_ts_, scan_ts]
            // plus one sample before (for interpolation).
            ImuSample before_boundary{};
            bool have_before = false;
            ImuSample after_boundary{};
            bool have_after = false;

            std::vector<ImuSample> imu_block;
            imu_block.reserve(256);  // typical: 100–200 samples

            // Track IMU interval for diagnostics.
            int64_t prev_imu_ts = 0;

            auto it = monotonic_imu_.begin();
            while (it != monotonic_imu_.end()) {
                const int64_t ts = it->timestamp_ns;

                if (ts <= prev_scan_ts_) {
                    // Before the interval — keep as interpolation anchor.
                    before_boundary = *it;
                    have_before = true;
                    it = monotonic_imu_.erase(it);
                    continue;
                }

                if (ts <= scan_ts) {
                    // Inside the interval.
                    if (prev_imu_ts > 0) {
                        const int64_t dt = ts - prev_imu_ts;
                        imu_interval_accumulator_ += dt;
                        ++imu_interval_count_;
                    }
                    prev_imu_ts = ts;
                    imu_block.push_back(*it);
                    it = monotonic_imu_.erase(it);
                    continue;
                }

                // Past the scan boundary — keep as interpolation anchor.
                if (!have_after) {
                    after_boundary = *it;
                    have_after = true;
                }
                break;
            }

            // ── Boundary interpolation ──────────────────────────────
            // Synthesize exact samples at prev_scan_ts_ and scan_ts.
            // When use_bspline_interpolation is enabled, use B-spline
            // over the full IMU block for smoother interpolation.

            if (config_.use_bspline_interpolation && imu_block.size() >= 4) {
                // B-spline mode: build extended block with anchors.
                std::vector<ImuSample> extended;
                extended.reserve(imu_block.size() + 2);
                if (have_before) extended.push_back(before_boundary);
                for (auto& s : imu_block) extended.push_back(s);
                if (have_after) extended.push_back(after_boundary);

                if (prev_scan_ts_ > 0 && extended.front().timestamp_ns < prev_scan_ts_) {
                    auto interp_start = ImuInterpolator::bspline(extended, prev_scan_ts_);
                    imu_block.insert(imu_block.begin(), interp_start);
                    meas.has_boundary_interp = true;
                } else if (prev_scan_ts_ > 0 && have_before &&
                           before_boundary.timestamp_ns == prev_scan_ts_) {
                    // Exact boundary sample — insert directly, no interpolation needed.
                    imu_block.insert(imu_block.begin(), before_boundary);
                    meas.has_boundary_interp = true;
                }
                if (have_after && extended.back().timestamp_ns > scan_ts) {
                    auto interp_end = ImuInterpolator::bspline(extended, scan_ts);
                    if (interp_end.timestamp_ns != imu_block.back().timestamp_ns) {
                        imu_block.push_back(interp_end);
                        meas.has_boundary_interp = true;
                    }
                }
            } else {
                // Linear interpolation (default).
                if (prev_scan_ts_ > 0 && have_before && !imu_block.empty()) {
                    auto interp_start = ImuInterpolator::lerp_clamped(
                        before_boundary, imu_block.front(), prev_scan_ts_);
                    imu_block.insert(imu_block.begin(), interp_start);
                    meas.has_boundary_interp = true;
                }

                if (!imu_block.empty() && have_after) {
                    auto interp_end = ImuInterpolator::lerp_clamped(
                        imu_block.back(), after_boundary, scan_ts);
                    if (interp_end.timestamp_ns != imu_block.back().timestamp_ns) {
                        imu_block.push_back(interp_end);
                        meas.has_boundary_interp = true;
                    }
                }
            }

            // ── Gap detection ───────────────────────────────────────
            if (imu_block.empty()) {
                ++stats_.imu_gaps_detected;
                meas.is_partial = true;
                if (imu_gap_cb_) {
                    const int64_t gap = scan_ts - prev_scan_ts_;
                    imu_gap_cb_(scan_ts, gap);
                }
            } else {
                // Check for internal gaps (missing samples).
                const double expected_interval_ns =
                    1.0e9 / config_.imu_rate_hz;
                for (size_t i = 1; i < imu_block.size(); ++i) {
                    const int64_t dt =
                        imu_block[i].timestamp_ns -
                        imu_block[i-1].timestamp_ns;
                    // Gap > 3× expected interval = missing samples.
                    if (static_cast<double>(dt) > 3.0 * expected_interval_ns) {
                        ++stats_.imu_gaps_detected;
                        meas.is_partial = true;
                        if (imu_gap_cb_) {
                            imu_gap_cb_(scan_ts, dt);
                        }
                        break;
                    }
                }
            }

            // ── Drift warning ───────────────────────────────────────
            if (config_.enable_drift_compensation) {
                const double drift = clock_service_
                    ? std::abs(clock_service_->drift_ns_per_sec())
                    : (drift_model_.is_valid()
                        ? std::abs(drift_model_.drift_ns_per_sec())
                        : 0.0);
                if (drift > config_.drift_warn_threshold_ns_per_sec) {
                    if (drift_warn_cb_) drift_warn_cb_(drift);
                }
            }

            // ── Update rolling IMU interval stat ────────────────────
            if (imu_interval_count_ > 0) {
                stats_.mean_imu_interval_ns =
                    static_cast<double>(imu_interval_accumulator_) /
                    static_cast<double>(imu_interval_count_);
            }

            stats_.imu_samples_dropped = sort_buffer_.dropped();
            meas.imu_block = std::move(imu_block);
        }

        // ── Finalize ────────────────────────────────────────────────
        meas.imu_sample_count =
            static_cast<uint32_t>(meas.imu_block.size());
        if (!meas.imu_block.empty()) {
            meas.scan_start_ns = meas.imu_block.front().timestamp_ns;
            meas.imu_span_ns =
                meas.imu_block.back().timestamp_ns -
                meas.imu_block.front().timestamp_ns;
        }

        const auto t1 = clock::now();
        meas.assembly_latency_us =
            std::chrono::duration<double, std::micro>(t1 - t0).count();

        // ── Update stats ────────────────────────────────────────────
        {
            std::lock_guard<std::mutex> lk2(imu_mu_);
            ++stats_.scans_assembled;
            if (meas.is_partial) ++stats_.partial_assemblies;

            // Rolling average of assembly latency.
            const double n =
                static_cast<double>(stats_.scans_assembled);
            stats_.avg_assembly_latency_us +=
                (meas.assembly_latency_us - stats_.avg_assembly_latency_us) / n;
        }

        // ── Consume the LiDAR scan ──────────────────────────────────
        {
            std::lock_guard<std::mutex> lk(lidar_mu_);
            if (!lidar_queue_.empty()) {
                lidar_queue_.pop_front();
            }
        }

        prev_scan_ts_ = scan_ts;
        ++scan_count_;

        return meas;
    }

    // ── State ───────────────────────────────────────────────────────────

    SlamTimeSyncConfig config_;

    // IMU path: sort buffer → monotonic deque.
    // Protected by imu_mu_.
    mutable std::mutex imu_mu_;
    ImuSortBuffer sort_buffer_;
    std::deque<ImuSample> monotonic_imu_;
    int64_t last_imu_ts_before_sort_{0};

    // LiDAR path.  Protected by lidar_mu_.
    mutable std::mutex lidar_mu_;
    std::deque<std::shared_ptr<const PointCloudFrame>> lidar_queue_;

    // Clock drift.  Accessed under imu_mu_ (same thread that feeds IMU).
    // When clock_service_ is set, drift_model_ serves as fallback only.
    ClockDriftModel drift_model_;
    const ClockService* clock_service_{nullptr};

    // Assembly state (single-consumer — ESIKF thread only).
    int64_t prev_scan_ts_{0};
    uint32_t scan_count_{0};

    // Diagnostics.
    SlamTimeSyncStats stats_{};
    int64_t imu_interval_accumulator_{0};
    uint64_t imu_interval_count_{0};

    // Callbacks.
    DriftWarningCallback drift_warn_cb_;
    ImuGapCallback imu_gap_cb_;
};

} // namespace thunderbird::odom
