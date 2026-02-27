// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Evaluation Framework: Resource Monitor
// ─────────────────────────────────────────────────────────────────────────────
//
// Samples CPU% and peak RSS at ~10 Hz in a background thread.
// Linux: reads /proc/self/stat and /proc/self/statm.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <atomic>
#include <cstddef>
#include <mutex>
#include <thread>
#include <vector>

namespace eval {

struct ResourceSample {
    double wall_time_s{0};
    double cpu_pct{0};
    size_t rss_bytes{0};
};

/// Lightweight CPU/memory sampler.  Start before replay, stop after.
///
/// Thread safety:
///   start()/stop() must be called from the main thread.
///   peakRss(), avgCpu(), and samples() are safe to call from any
///   thread (including while the monitor is running).
class ResourceMonitor {
public:
    void start();
    void stop();

    /// Return a snapshot copy of all samples collected so far.
    std::vector<ResourceSample> samples() const;

    /// Peak RSS observed across all samples.
    size_t peakRss() const;

    /// Average CPU% across all samples.
    double avgCpu() const;

private:
    std::atomic<bool>           running_{false};
    std::thread                 thread_;
    mutable std::mutex          mu_;       ///< Protects samples_.
    std::vector<ResourceSample> samples_;  ///< Guarded by mu_.

    void run();
};

} // namespace eval
