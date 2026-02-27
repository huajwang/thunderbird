// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — RuntimeProfiler: Module-Level Performance Instrumentation
// ─────────────────────────────────────────────────────────────────────────────
//
// Low-overhead profiler that lives inside AcmeSlamEngine::Impl.  Measures:
//
//   • Per-module CPU time (imu_propagate, deskew, esikf_update,
//     ikd_insert, ikd_rebalance, scan_total, worker_idle).
//   • Worker thread utilisation (busy vs idle).
//   • Process RSS (peak and current).
//   • Percentile latency breakdown (p50/p95/p99/max) per module.
//
// All timings use std::chrono::steady_clock (monotonic, ~ns precision).
// The profiler is single-threaded (worker thread only) — no atomics needed
// for internal counters.  The snapshot() method copies data for cross-thread
// consumption.
//
// Overhead:
//   • 2× clock_gettime per scope (~20–40 ns each on x86).
//   • Zero heap allocation in the hot path.
//
// Usage:
//
//   RuntimeProfiler prof;
//
//   // RAII scope:
//   {
//       RuntimeProfiler::Scope s(prof, ProfileModule::EsikfUpdate);
//       esikf.update(cloud, tree, cfg);
//   }
//
//   // Query:
//   ProfileSnapshot snap = prof.snapshot();
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/odom/slam_types.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace thunderbird::odom {

// ═════════════════════════════════════════════════════════════════════════════
//  RollingStats — O(1)-per-sample accumulator with percentile support
// ═════════════════════════════════════════════════════════════════════════════

class RollingStats {
public:
    void add(double val) noexcept {
        ++n_;
        sum_ += val;
        if (val < min_) min_ = val;
        if (val > max_) max_ = val;
        // Circular history buffer for percentile queries.
        history_[history_pos_ % kHistorySize] = val;
        ++history_pos_;
    }

    size_t  count()   const noexcept { return n_; }
    double  sum()     const noexcept { return sum_; }
    double  mean()    const noexcept { return n_ > 0 ? sum_ / static_cast<double>(n_) : 0.0; }
    double  min_val() const noexcept { return n_ > 0 ? min_ : 0.0; }
    double  max_val() const noexcept { return max_; }

    /// Compute percentile from the circular history buffer.
    /// @param pct  Percentile [0, 100].
    double percentile(double pct) const {
        if (n_ == 0) return 0.0;
        const size_t len = std::min(n_, kHistorySize);
        std::vector<double> sorted(history_.data(),
                                   history_.data() + len);
        std::sort(sorted.begin(), sorted.end());
        const size_t idx = static_cast<size_t>(
            pct / 100.0 * static_cast<double>(len - 1));
        return sorted[std::min(idx, len - 1)];
    }

    void reset() noexcept {
        n_ = 0;
        sum_ = 0.0;
        min_ = 1e18;
        max_ = 0.0;
        history_pos_ = 0;
    }

private:
    static constexpr size_t kHistorySize = 8192;

    size_t n_{0};
    double sum_{0.0};
    double min_{1e18};
    double max_{0.0};
    std::array<double, kHistorySize> history_{};
    size_t history_pos_{0};
};

// ═════════════════════════════════════════════════════════════════════════════
//  RuntimeProfiler
// ═════════════════════════════════════════════════════════════════════════════

class RuntimeProfiler {
public:
    using Clock     = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    // ── RAII Scoped Timer ───────────────────────────────────────────────

    class Scope {
    public:
        Scope(RuntimeProfiler& prof, ProfileModule mod) noexcept
            : prof_(prof), mod_(mod), t0_(Clock::now()) {}

        ~Scope() { prof_.record(mod_, t0_); }

        // Non-copyable, non-movable.
        Scope(const Scope&)            = delete;
        Scope& operator=(const Scope&) = delete;

    private:
        RuntimeProfiler& prof_;
        ProfileModule     mod_;
        TimePoint         t0_;
    };

    // ── Construction ────────────────────────────────────────────────────

    RuntimeProfiler() noexcept
        : start_time_(Clock::now()),
          last_idle_end_(Clock::now()) {}

    // ── Recording (hot path — called from worker thread) ────────────────

    /// Record a completed interval for `mod` that started at `t0`.
    void record(ProfileModule mod, TimePoint t0) noexcept {
        const auto t1 = Clock::now();
        const double us = std::chrono::duration<double, std::micro>(
            t1 - t0).count();
        modules_[static_cast<size_t>(mod)].add(us);

        if (mod != ProfileModule::WorkerIdle) {
            total_busy_ns_ += std::chrono::duration_cast<
                std::chrono::nanoseconds>(t1 - t0).count();
        }
    }

    /// Mark the start of an idle wait (call before cv.wait).
    void markIdleStart() noexcept {
        idle_start_ = Clock::now();
    }

    /// Mark the end of an idle wait (call after cv.wait returns).
    void markIdleEnd() noexcept {
        const auto now = Clock::now();
        const double us = std::chrono::duration<double, std::micro>(
            now - idle_start_).count();
        modules_[static_cast<size_t>(ProfileModule::WorkerIdle)].add(us);
        total_idle_ns_ += std::chrono::duration_cast<
            std::chrono::nanoseconds>(now - idle_start_).count();
        last_idle_end_ = now;
    }

    /// Sample current process RSS (call periodically, e.g. once per scan).
    void sampleMemory() noexcept;

    // ── Snapshot (called from any thread — copies data) ─────────────────

    /// Build a ProfileSnapshot with full statistics.
    ProfileSnapshot snapshot() const;

    // ── Reset ───────────────────────────────────────────────────────────

    void reset() noexcept {
        for (auto& m : modules_) m.reset();
        total_busy_ns_ = 0;
        total_idle_ns_ = 0;
        peak_rss_      = 0;
        current_rss_   = 0;
        start_time_    = Clock::now();
        last_idle_end_ = start_time_;
    }

private:
    // Module names (indexed by ProfileModule enum).
    static constexpr const char* kModuleNames[kProfileModuleCount] = {
        "imu_propagate",
        "deskew",
        "esikf_update",
        "ikd_insert",
        "ikd_rebalance",
        "scan_total",
        "worker_idle",
    };

    std::array<RollingStats, kProfileModuleCount> modules_{};

    TimePoint start_time_;
    TimePoint idle_start_{};
    TimePoint last_idle_end_;

    int64_t total_busy_ns_{0};
    int64_t total_idle_ns_{0};

    size_t peak_rss_{0};
    size_t current_rss_{0};
};

} // namespace thunderbird::odom
