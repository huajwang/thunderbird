// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Evaluation Framework: Evaluator
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "metrics.h"
#include "pose_recorder.h"
#include "resource_monitor.h"
#include "trajectory_alignment.h"

#include <string>

namespace eval {

class Evaluator {
public:
    struct Config {
        double rpe_delta_m            = 100.0;   // RPE segment length
        bool   align_trajectories     = true;    // SE(3) alignment before ATE
        bool   estimate_scale         = false;   // Sim(3) scale recovery
        bool   outlier_rejection      = false;   // iterative outlier removal
        double outlier_threshold_sigma = 3.0;    // sigma threshold for outliers
    };

    Evaluator() = default;
    explicit Evaluator(Config cfg) : cfg_(cfg) {}

    /// Compute all metrics from recorded poses + resource samples.
    MetricSet compute(const PoseRecorder& recorder,
                      const ResourceMonitor& resources,
                      const std::vector<FrameTiming>& timings) const;

    /// Output writers.
    static void writeCsv(const MetricSet& m, const std::string& path);
    static void writeJson(const MetricSet& m, const std::string& path);
    static void writeTumTrajectory(const std::vector<StampedPose>& est,
                                   const std::string& path);
    static void writeTumTrajectory(const std::vector<GtPose>& gt,
                                   const std::string& path);

private:
    Config cfg_;
};

} // namespace eval
