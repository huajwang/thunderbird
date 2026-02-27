// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Evaluation Framework: Performance Timer
// ─────────────────────────────────────────────────────────────────────────────
//
// Lightweight scoped timer and manual lap timer for instrumenting the
// evaluation pipeline.  Separate from the SDK's SlamProfiler — this is
// for timing the *evaluation harness itself* (disk I/O, feed latency,
// drain wait), not the internal SLAM components.
//
// All timings use std::chrono::steady_clock (monotonic).
//
// Usage:
//
//   PerfTimer timer;
//
//   timer.start("load");
//   auto cloud = load_bin_file(path);
//   double load_ms = timer.stop_ms();
//
//   timer.start("feed");
//   engine.feedPointCloud(cloud);
//   double feed_ms = timer.stop_ms();
//
//   // Or with RAII scope:
//   {
//       ScopedPerfTimer t(timer, "drain");
//       engine.drainOutputs(outputs);
//   }
//   double drain_ms = timer.total_ms("drain");
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace eval {

// ═════════════════════════════════════════════════════════════════════════════
//  PerfTimer — manual start/stop timer with named laps
// ═════════════════════════════════════════════════════════════════════════════

class PerfTimer {
public:
    using Clock     = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    /// Start (or restart) a named timer.
    void start(const std::string& name) {
        const auto now_tp = Clock::now();
        starts_[name] = now_tp;
        last_name_  = name;
        last_start_ = now_tp;
    }

    /// Stop the named timer and return elapsed milliseconds.
    /// Also accumulates into per-name totals.
    double stop_ms(const std::string& name) {
        auto it = starts_.find(name);
        if (it == starts_.end()) return 0.0;
        const auto now_tp = Clock::now();
        const double ms = std::chrono::duration<double, std::milli>(now_tp - it->second).count();
        totals_[name] += ms;
        counts_[name]++;
        last_stop_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
            now_tp - it->second).count();
        return ms;
    }

    /// Stop the named timer and return elapsed nanoseconds (full precision).
    int64_t stop_ns(const std::string& name) {
        auto it = starts_.find(name);
        if (it == starts_.end()) return 0;
        const auto now_tp = Clock::now();
        const int64_t ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            now_tp - it->second).count();
        totals_[name] += static_cast<double>(ns) / 1.0e6;
        counts_[name]++;
        return ns;
    }

    /// Return the nanosecond value from the last stop_ms() call.
    /// Use this to get ns precision without a second clock read.
    int64_t last_stop_ns() const { return last_stop_ns_; }

    /// Stop the most recently started timer (convenience for single-lap use).
    double stop_ms() {
        if (last_name_.empty()) return 0.0;
        return stop_ms(last_name_);
    }

    /// Stop the most recently started timer, return nanoseconds.
    int64_t stop_ns() {
        if (last_name_.empty()) return 0;
        return stop_ns(last_name_);
    }

    /// Start and remember as "last" (so stop_ms() with no arg works).
    void start() {
        last_start_ = Clock::now();
        last_name_  = "__last__";
        starts_[last_name_] = last_start_;
    }

    /// Total accumulated time for a named timer (ms).
    double total_ms(const std::string& name) const {
        auto it = totals_.find(name);
        return it != totals_.end() ? it->second : 0.0;
    }

    /// Number of laps recorded for a named timer.
    size_t count(const std::string& name) const {
        auto it = counts_.find(name);
        return it != counts_.end() ? it->second : 0;
    }

    /// Average time per lap (ms).
    double avg_ms(const std::string& name) const {
        auto c = count(name);
        return c > 0 ? total_ms(name) / static_cast<double>(c) : 0.0;
    }

    /// Reset all timers.
    void reset() {
        starts_.clear();
        totals_.clear();
        counts_.clear();
        last_name_.clear();
    }

    /// Wall-clock now.
    static TimePoint now() { return Clock::now(); }

    /// Milliseconds since a given time point.
    static double elapsed_since_ms(TimePoint tp) {
        return std::chrono::duration<double, std::milli>(Clock::now() - tp).count();
    }

    /// Nanoseconds since a given time point (full precision, no float truncation).
    static int64_t elapsed_since_ns(TimePoint tp) {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now() - tp).count();
    }

private:
    std::unordered_map<std::string, TimePoint> starts_;
    std::unordered_map<std::string, double>    totals_;
    std::unordered_map<std::string, size_t>    counts_;
    TimePoint   last_start_;
    std::string last_name_;
    int64_t     last_stop_ns_{0};
};

// ═════════════════════════════════════════════════════════════════════════════
//  ScopedPerfTimer — RAII helper that calls start/stop automatically
// ═════════════════════════════════════════════════════════════════════════════

class ScopedPerfTimer {
public:
    ScopedPerfTimer(PerfTimer& timer, const std::string& name)
        : timer_(timer), name_(name) {
        timer_.start(name_);
    }

    ~ScopedPerfTimer() {
        elapsed_ms_ = timer_.stop_ms(name_);
    }

    // Non-copyable, non-movable.
    ScopedPerfTimer(const ScopedPerfTimer&) = delete;
    ScopedPerfTimer& operator=(const ScopedPerfTimer&) = delete;

    /// Get elapsed time after destruction (use via pointer from outer scope).
    double elapsed_ms() const { return elapsed_ms_; }

private:
    PerfTimer&  timer_;
    std::string name_;
    double      elapsed_ms_{0};
};

// ═════════════════════════════════════════════════════════════════════════════
//  ReplayRateLogger — tracks and logs replay throughput
// ═════════════════════════════════════════════════════════════════════════════

class ReplayRateLogger {
public:
    explicit ReplayRateLogger(double log_interval_s = 5.0)
        : interval_s_(log_interval_s) {}

    /// Call once before the replay loop.
    void start() {
        start_    = PerfTimer::now();
        last_log_ = start_;
        frames_since_log_ = 0;
        total_frames_     = 0;
    }

    /// Call after each LiDAR frame is fed.  Returns true if a log line
    /// was printed (caller can use this for progress bars etc.).
    bool tick(size_t frame_index, size_t total_frames_expected);

    /// Call after the replay loop ends.  Prints final summary.
    void finish();

    /// Overall replay rate (frames/sec).
    double overall_fps() const;

private:
    double interval_s_{5.0};
    PerfTimer::TimePoint start_;
    PerfTimer::TimePoint last_log_;
    size_t frames_since_log_{0};
    size_t total_frames_{0};
};

} // namespace eval
