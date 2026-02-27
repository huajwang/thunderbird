// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Evaluation Framework: Metrics Types
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace eval {

// ═════════════════════════════════════════════════════════════════════════════
//  Individual metric results
// ═════════════════════════════════════════════════════════════════════════════

struct AteResult {
    double rmse_m{0};
    double mean_m{0};
    double max_m{0};
    double median_m{0};
};

struct RpeResult {
    double trans_rmse_m{0};     // translational RPE (metres)
    double rot_rmse_deg{0};     // rotational RPE (degrees)
    double delta_m{100.0};      // evaluation segment length
};

struct DriftResult {
    double drift_pct{0};        // % of total distance
    double drift_per_100m{0};   // metres drift per 100 m travelled
};

struct RuntimeResult {
    double avg_ms{0};           // average ms per LiDAR frame
    double p95_ms{0};
    double max_ms{0};
    size_t num_frames{0};
};

struct ResourceResult {
    double avg_cpu_pct{0};
    size_t peak_rss_bytes{0};
};

// ═════════════════════════════════════════════════════════════════════════════
//  Per-frame timing record (for CSV dump)
// ═════════════════════════════════════════════════════════════════════════════

struct FrameTiming {
    size_t  frame_index{0};
    int64_t timestamp_ns{0};
    double  load_ms{0};         // time to read .bin from disk
    double  feed_ms{0};         // feedImu + feedPointCloud
    double  wait_ms{0};         // wall time polling for engine output
    double  total_ms{0};        // load + feed + wait

    // ── Nanosecond-precision variants (no float truncation) ─────────────
    int64_t load_ns{0};
    int64_t feed_ns{0};
    int64_t wait_ns{0};
    int64_t total_ns{0};

    bool    output_received{false}; // true if engine produced output within timeout
};

// ═════════════════════════════════════════════════════════════════════════════
//  MetricSet — all metrics for a single evaluation run
// ═════════════════════════════════════════════════════════════════════════════

struct MetricSet {
    std::string    dataset_name;
    AteResult      ate;
    RpeResult      rpe;
    DriftResult    drift;
    RuntimeResult  runtime;
    ResourceResult resources;

    size_t num_poses{0};
    double sequence_duration_s{0};
    double total_distance_m{0};

    std::vector<FrameTiming> frame_timings;
};

} // namespace eval
