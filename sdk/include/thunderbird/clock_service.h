// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Clock Service (Unified Clock Authority)
// ─────────────────────────────────────────────────────────────────────────────
//
// Provides a single, shared clock-synchronization service for the entire SDK.
// Replaces scattered drift-tracking logic in SlamTimeSync / TimeSyncEngine.
//
// Core algorithm:
//   • OLS linear regression:  host = α·hw + β
//   • Outlier rejection via MAD (Median Absolute Deviation)
//   • Jump detection with confirmation count
//   • Lock-free seqlock reads for hw_to_host() / unified_timestamp()
//   • Optional PPS anchoring (Car profile)
//
// Threading model:
//   • observe() / observe_pps() — called from I/O thread only (single writer)
//   • hw_to_host() / unified_timestamp() — called from any thread (readers)
//   • diagnostics() — called from any thread (reader, mutex-protected)
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstring>

namespace thunderbird {

// ── Clock synchronization profile ───────────────────────────────────────────

enum class ClockProfile : uint8_t {
    /// Car: GNSS-disciplined device clock.  Expect sub-μs HW timestamps
    /// with occasional jump (leap second, GNSS reacquisition).
    Car = 0,

    /// Drone: Free-running crystal oscillator on device.  No absolute
    /// time reference.  Drift 1–50 μs/s typical (MEMS XO).
    Drone = 1,
};

// ── Configuration ───────────────────────────────────────────────────────────

struct ClockServiceConfig {
    ClockProfile profile{ClockProfile::Drone};

    /// OLS window size for offset estimation.
    size_t ols_window{100};

    /// Jump detection threshold (ns).
    int64_t jump_threshold_ns{50'000'000};  // 50 ms

    /// Consecutive jump-flagged samples before confirmed jump.
    uint32_t jump_confirm_count{5};

    /// Maximum acceptable drift rate (ns/s) before warning.
    double drift_warn_threshold_ns_per_sec{1'000'000.0};  // 1 ms/s

    /// Enable PPS anchoring (Car profile only).
    bool enable_pps{false};

    /// PPS expected interval (ns).
    int64_t pps_interval_ns{1'000'000'000};

    /// PPS tolerance: max jitter before pulse is rejected.
    int64_t pps_tolerance_ns{10'000'000};  // 10 ms

    /// MAD outlier rejection k-factor (multiples of MAD).
    double mad_k_factor{3.0};

    /// Observation decimation factor (accept every Nth observation).
    /// Set to 1 to accept all observations.
    uint32_t decimate_factor{1};
};

// ── Diagnostic events ───────────────────────────────────────────────────────

enum class ClockEvent : uint8_t {
    Calibrated,      ///< First valid offset estimate available
    DriftWarning,    ///< Drift exceeds threshold
    TimeJump,        ///< Confirmed time jump detected, model reset
    PpsLocked,       ///< PPS anchor acquired
    PpsLost,         ///< PPS signal lost (N misses)
    ModelReset,      ///< Model was reset
};

struct ClockDiagnostics {
    double   offset_ns{0};           ///< Current estimated offset (host − hw)
    double   drift_ns_per_sec{0};    ///< Current drift rate
    double   offset_stddev_ns{0};    ///< Uncertainty of offset estimate
    uint64_t observations{0};        ///< Total (hw, host) pairs ingested
    uint64_t jumps_detected{0};      ///< Total confirmed time jumps
    uint64_t outliers_rejected{0};   ///< Samples rejected by outlier filter
    bool     calibrated{false};      ///< True once model has enough samples
    bool     pps_locked{false};      ///< True if PPS anchor is active
};

using ClockEventCallback = std::function<void(ClockEvent, const ClockDiagnostics&)>;

// ─────────────────────────────────────────────────────────────────────────────
// ClockService — THE single clock authority for the entire SDK
// ─────────────────────────────────────────────────────────────────────────────

class ClockService {
public:
    explicit ClockService(const ClockServiceConfig& config = {})
        : config_(config)
    {
        hw_samples_.reserve(config_.ols_window);
        host_samples_.reserve(config_.ols_window);
        residuals_.reserve(config_.ols_window);
    }

    // ── Observation ingestion (called from I/O thread) ──────────────────

    /// Record a (hardware_timestamp, host_timestamp) pair.
    void observe(int64_t hw_ns, int64_t host_ns) {
        if (hw_ns == 0) return;  // No hardware timestamps available

        ++total_observations_;

        // Decimation: skip unless this is an Nth observation.
        if (config_.decimate_factor > 1 &&
            (total_observations_ % config_.decimate_factor) != 0) {
            return;
        }

        {
            std::lock_guard lk(write_mu_);
            observe_locked(hw_ns, host_ns);
        }
        fire_pending_events();
    }

    /// Record a PPS edge arrival (Car profile only).
    void observe_pps(int64_t host_ns, int64_t pps_hw_ns = 0) {
        if (!config_.enable_pps) return;

        {
            std::lock_guard lk(write_mu_);

            bool jittery = false;
            if (last_pps_host_ns_ != 0) {
                int64_t interval = host_ns - last_pps_host_ns_;
                int64_t error = std::abs(interval - config_.pps_interval_ns);
                jittery = (error > config_.pps_tolerance_ns);
            }

            if (jittery) {
                // Reject jittery pulse.
                ++pps_miss_count_;
                if (pps_miss_count_ >= 3 && diag_.pps_locked) {
                    diag_.pps_locked = false;
                    emit(ClockEvent::PpsLost);
                }
                last_pps_host_ns_ = host_ns;
            } else {
                // Valid PPS pulse.
                pps_miss_count_ = 0;
                last_pps_host_ns_ = host_ns;

                // Quantize to nearest second boundary.
                int64_t pps_epoch = (host_ns + 500'000'000LL) /
                                    1'000'000'000LL * 1'000'000'000LL;
                pps_offset_ns_ = pps_epoch - host_ns;

                // Apply PPS correction to β.
                if (diag_.calibrated) {
                    beta_ += pps_offset_ns_ * 0.1;  // Smooth correction
                    publish_model(alpha_, beta_, true);
                }

                ++pps_lock_count_;
                if (pps_lock_count_ >= 3 && !diag_.pps_locked) {
                    diag_.pps_locked = true;
                    emit(ClockEvent::PpsLocked);
                }

                // Also observe as clock pair if hw timestamp provided.
                // Call observe_locked() directly to avoid recursive mutex.
                if (pps_hw_ns != 0) {
                    ++total_observations_;
                    if (config_.decimate_factor <= 1 ||
                        (total_observations_ % config_.decimate_factor) == 0) {
                        observe_locked(pps_hw_ns, host_ns);
                    }
                }
            }
        }
        fire_pending_events();
    }

    // ── Timestamp conversion (called from any thread, lock-free) ────────

    /// Convert hardware timestamp to host clock domain.
    /// Before calibration, returns hw_ns unchanged.
    [[nodiscard]] int64_t hw_to_host(int64_t hw_ns) const noexcept {
        double a, b;
        bool cal;
        read_model(a, b, cal);
        if (!cal) return hw_ns;
        return static_cast<int64_t>(a * static_cast<double>(hw_ns) + b);
    }

    /// Convert host timestamp to hardware clock domain (inverse).
    [[nodiscard]] int64_t host_to_hw(int64_t host_ns) const noexcept {
        double a, b;
        bool cal;
        read_model(a, b, cal);
        if (!cal || a == 0.0) return host_ns;
        return static_cast<int64_t>(
            (static_cast<double>(host_ns) - b) / a);
    }

    /// Return the best unified timestamp.
    /// If calibrated: hw_to_host(hw_ns) (more stable).
    /// If not calibrated: host_ns (best we have).
    [[nodiscard]] int64_t unified_timestamp(int64_t hw_ns,
                                            int64_t host_ns) const noexcept {
        double a, b;
        bool cal;
        read_model(a, b, cal);
        if (!cal) return host_ns;
        return static_cast<int64_t>(a * static_cast<double>(hw_ns) + b);
    }

    // ── Diagnostics ─────────────────────────────────────────────────────

    [[nodiscard]] ClockDiagnostics diagnostics() const {
        std::lock_guard lk(write_mu_);
        return diag_;
    }

    [[nodiscard]] bool is_calibrated() const noexcept {
        return seq_calibrated_.load(std::memory_order_acquire);
    }

    [[nodiscard]] double drift_ns_per_sec() const noexcept {
        double a, b;
        bool cal;
        read_model(a, b, cal);
        if (!cal) return 0.0;
        return (a - 1.0) * 1e9;
    }

    void on_event(ClockEventCallback cb) {
        std::lock_guard lk(write_mu_);
        event_cb_ = std::move(cb);
    }

    // ── Lifecycle ───────────────────────────────────────────────────────

    void reset() {
        {
            std::lock_guard lk(write_mu_);
            hw_samples_.clear();
            host_samples_.clear();
            residuals_.clear();
            alpha_ = 1.0;
            beta_ = 0.0;
            diag_ = {};
            total_observations_ = 0;
            jump_candidate_count_ = 0;
            jump_hw_buf_.clear();
            jump_host_buf_.clear();
            pps_miss_count_ = 0;
            pps_lock_count_ = 0;
            last_pps_host_ns_ = 0;
            pps_offset_ns_ = 0;
            publish_model(1.0, 0.0, false);
            emit(ClockEvent::ModelReset);
        }
        fire_pending_events();
    }

private:
    // ── Core observation logic (must be called with write_mu_ held) ─────

    void observe_locked(int64_t hw_ns, int64_t host_ns) {
        diag_.observations = total_observations_;

        if (!diag_.calibrated) {
            // Collecting phase: fill the window until we have enough.
            add_sample(hw_ns, host_ns);
            if (hw_samples_.size() >= 3) {
                recompute_ols();
                diag_.calibrated = true;
                publish_model(alpha_, beta_, true);
                emit(ClockEvent::Calibrated);
            }
            return;
        }

        // Calibrated: compute residual.
        double predicted = alpha_ * static_cast<double>(hw_ns) + beta_;
        double residual = static_cast<double>(host_ns) - predicted;

        // MAD-based outlier test (falls back to jump_threshold_ns when MAD≈0).
        if (residuals_.size() >= 3) {
            double mad = compute_mad();
            double threshold = (mad > 0)
                ? config_.mad_k_factor * mad
                : static_cast<double>(config_.jump_threshold_ns);
            if (std::abs(residual) > threshold) {
                // Outlier detected.
                ++diag_.outliers_rejected;
                ++jump_candidate_count_;

                // Store for potential re-seeding after confirmed jump.
                jump_hw_buf_.push_back(hw_ns);
                jump_host_buf_.push_back(host_ns);
                if (jump_hw_buf_.size() > config_.jump_confirm_count + 2) {
                    jump_hw_buf_.erase(jump_hw_buf_.begin());
                    jump_host_buf_.erase(jump_host_buf_.begin());
                }

                // Check for confirmed time jump.
                if (jump_candidate_count_ >= config_.jump_confirm_count) {
                    handle_time_jump();
                }
                return;
            }
        }

        // Normal sample: accept into OLS window.
        jump_candidate_count_ = 0;
        jump_hw_buf_.clear();
        jump_host_buf_.clear();
        add_sample(hw_ns, host_ns);
        recompute_ols();
        publish_model(alpha_, beta_, true);
        check_drift_warning();
    }

    // ── OLS regression ──────────────────────────────────────────────────

    void add_sample(int64_t hw_ns, int64_t host_ns) {
        if (hw_samples_.size() >= config_.ols_window) {
            hw_samples_.erase(hw_samples_.begin());
            host_samples_.erase(host_samples_.begin());
            if (!residuals_.empty())
                residuals_.erase(residuals_.begin());
        }
        hw_samples_.push_back(static_cast<double>(hw_ns));
        host_samples_.push_back(static_cast<double>(host_ns));
    }

    void recompute_ols() {
        const size_t n = hw_samples_.size();
        if (n < 2) return;

        // Compute means.
        double sum_x = 0, sum_y = 0;
        for (size_t i = 0; i < n; ++i) {
            sum_x += hw_samples_[i];
            sum_y += host_samples_[i];
        }
        double mean_x = sum_x / static_cast<double>(n);
        double mean_y = sum_y / static_cast<double>(n);

        // Compute slope (α) and intercept (β).
        double num = 0, den = 0;
        for (size_t i = 0; i < n; ++i) {
            double dx = hw_samples_[i] - mean_x;
            double dy = host_samples_[i] - mean_y;
            num += dx * dy;
            den += dx * dx;
        }

        if (den == 0) return;

        alpha_ = num / den;
        beta_ = mean_y - alpha_ * mean_x;

        // Compute residuals and std dev.
        residuals_.clear();
        double sum_sq = 0;
        for (size_t i = 0; i < n; ++i) {
            double pred = alpha_ * hw_samples_[i] + beta_;
            double r = host_samples_[i] - pred;
            residuals_.push_back(r);
            sum_sq += r * r;
        }

        diag_.offset_ns = beta_;
        diag_.drift_ns_per_sec = (alpha_ - 1.0) * 1e9;

        if (n > 2) {
            diag_.offset_stddev_ns =
                std::sqrt(sum_sq / static_cast<double>(n - 2));
        }
    }

    double compute_mad() const {
        if (residuals_.empty()) return 0.0;
        std::vector<double> sorted = residuals_;
        for (auto& v : sorted) v = std::abs(v);
        std::sort(sorted.begin(), sorted.end());
        // Median of absolute residuals.
        size_t mid = sorted.size() / 2;
        double median = (sorted.size() % 2 == 0)
            ? (sorted[mid - 1] + sorted[mid]) / 2.0
            : sorted[mid];
        // Scale to be consistent with σ for normal distribution.
        return median * 1.4826;
    }

    // ── Jump detection ──────────────────────────────────────────────────

    void handle_time_jump() {
        ++diag_.jumps_detected;

        // Re-seed the model with the recent rejected observations
        // (which represent the NEW clock state).
        hw_samples_.clear();
        host_samples_.clear();
        residuals_.clear();

        for (size_t i = 0; i < jump_hw_buf_.size(); ++i) {
            hw_samples_.push_back(static_cast<double>(jump_hw_buf_[i]));
            host_samples_.push_back(static_cast<double>(jump_host_buf_[i]));
        }

        jump_candidate_count_ = 0;
        jump_hw_buf_.clear();
        jump_host_buf_.clear();

        if (hw_samples_.size() >= 3) {
            recompute_ols();
            publish_model(alpha_, beta_, true);
        } else {
            diag_.calibrated = false;
            publish_model(1.0, 0.0, false);
        }

        emit(ClockEvent::TimeJump);
    }

    // ── Drift warning ───────────────────────────────────────────────────

    void check_drift_warning() {
        double drift = std::abs(diag_.drift_ns_per_sec);
        if (drift > config_.drift_warn_threshold_ns_per_sec) {
            emit(ClockEvent::DriftWarning);
        }
    }

    // ── Seqlock for lock-free reads ─────────────────────────────────────

    void publish_model(double alpha, double beta, bool calibrated) {
        seq_.fetch_add(1, std::memory_order_release);  // odd = writing
        seq_alpha_.store(alpha, std::memory_order_relaxed);
        seq_beta_.store(beta, std::memory_order_relaxed);
        seq_calibrated_.store(calibrated, std::memory_order_relaxed);
        seq_.fetch_add(1, std::memory_order_release);  // even = safe
    }

    void read_model(double& alpha, double& beta, bool& calibrated) const noexcept {
        uint32_t s;
        do {
            s = seq_.load(std::memory_order_acquire);
            if (s & 1) continue;  // write in progress — spin
            alpha = seq_alpha_.load(std::memory_order_relaxed);
            beta = seq_beta_.load(std::memory_order_relaxed);
            calibrated = seq_calibrated_.load(std::memory_order_relaxed);
        } while (seq_.load(std::memory_order_acquire) != s);
    }

    // ── Event emission ──────────────────────────────────────────────────

    /// Buffer an event for deferred delivery (must be called under write_mu_).
    void emit(ClockEvent event) {
        pending_events_.push_back(event);
    }

    /// Fire all buffered events outside the lock to avoid deadlocks
    /// if the callback re-enters ClockService (e.g. calls diagnostics()).
    void fire_pending_events() {
        ClockEventCallback cb;
        ClockDiagnostics snap;
        std::vector<ClockEvent> events;
        {
            std::lock_guard lk(write_mu_);
            if (pending_events_.empty()) return;
            events.swap(pending_events_);
            cb = event_cb_;
            snap = diag_;
        }
        if (cb) {
            for (auto e : events) cb(e, snap);
        }
    }

    // ── Members ─────────────────────────────────────────────────────────

    ClockServiceConfig config_;

    // OLS samples (write-side only, protected by write_mu_).
    std::vector<double> hw_samples_;
    std::vector<double> host_samples_;
    std::vector<double> residuals_;
    double alpha_{1.0};
    double beta_{0.0};

    // Diagnostics (write-side only, protected by write_mu_).
    ClockDiagnostics diag_{};
    uint64_t total_observations_{0};

    // Jump detection.
    uint32_t jump_candidate_count_{0};
    std::vector<int64_t> jump_hw_buf_;
    std::vector<int64_t> jump_host_buf_;

    // PPS state.
    uint32_t pps_miss_count_{0};
    uint32_t pps_lock_count_{0};
    int64_t  last_pps_host_ns_{0};
    double   pps_offset_ns_{0};

    // Seqlock atomics for lock-free reader path.
    mutable std::atomic<uint32_t> seq_{0};
    std::atomic<double> seq_alpha_{1.0};
    std::atomic<double> seq_beta_{0.0};
    std::atomic<bool>   seq_calibrated_{false};

    // Write mutex (protects all non-atomic state).
    mutable std::mutex write_mu_;

    // Event callback + pending event buffer (protected by write_mu_).
    ClockEventCallback event_cb_;
    std::vector<ClockEvent> pending_events_;
};

} // namespace thunderbird
