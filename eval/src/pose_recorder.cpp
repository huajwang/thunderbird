// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Evaluation Framework: Pose Recorder Implementation
// ─────────────────────────────────────────────────────────────────────────────

#include "eval/pose_recorder.h"

#include <algorithm>

namespace eval {

void PoseRecorder::record(const SlamOutput& output) {
    StampedPose sp;
    sp.timestamp_ns = output.pose.timestamp_ns;
    sp.position     = output.pose.position;
    sp.quaternion   = output.pose.quaternion;

    std::lock_guard<std::mutex> lock(mu_);
    estimated_.push_back(sp);
}

void PoseRecorder::setGroundTruth(std::vector<GtPose> gt) {
    gt_ = std::move(gt);
}

std::vector<StampedPose> PoseRecorder::estimated() const {
    std::lock_guard<std::mutex> lock(mu_);
    return estimated_;
}

size_t PoseRecorder::size() const {
    std::lock_guard<std::mutex> lock(mu_);
    return estimated_.size();
}

void PoseRecorder::clear() {
    std::lock_guard<std::mutex> lock(mu_);
    estimated_.clear();
    gt_.clear();
}

} // namespace eval
