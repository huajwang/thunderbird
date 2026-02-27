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
#include <thread>
#include <vector>

namespace eval {

struct ResourceSample {
    double wall_time_s{0};
    double cpu_pct{0};
    size_t rss_bytes{0};
};

/// Lightweight CPU/memory sampler.  Start before replay, stop after.
class ResourceMonitor {
public:
    void start();
    void stop();

    const std::vector<ResourceSample>& samples() const { return samples_; }

    /// Peak RSS observed across all samples.
    size_t peakRss() const;

    /// Average CPU% across all samples.
    double avgCpu() const;

private:
    std::atomic<bool>          running_{false};
    std::thread                thread_;
    std::vector<ResourceSample> samples_;

    void run();
};

} // namespace eval
