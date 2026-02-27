// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Evaluation Framework: Pose Recorder
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "dataset_adapter.h"
#include "thunderbird/odom/slam_types.h"

#include <mutex>
#include <vector>

namespace eval {

using namespace thunderbird::odom;

struct StampedPose {
    int64_t              timestamp_ns{0};
    std::array<double,3> position{};
    std::array<double,4> quaternion{1,0,0,0};
};

/// Collects estimated poses from AcmeSlamEngine output.
/// Thread-safe: record() may be called from engine callback thread.
class PoseRecorder {
public:
    /// Record a pose from a SlamOutput.
    void record(const SlamOutput& output);

    /// Set ground truth trajectory for later evaluation.
    void setGroundTruth(std::vector<GtPose> gt);

    /// Access recorded data.
    std::vector<StampedPose> estimated() const;
    const std::vector<GtPose>& groundTruth() const { return gt_; }

    /// Number of estimated poses recorded.
    size_t size() const;

    void clear();

private:
    mutable std::mutex     mu_;
    std::vector<StampedPose> estimated_;
    std::vector<GtPose>      gt_;
};

} // namespace eval
