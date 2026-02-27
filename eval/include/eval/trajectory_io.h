// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Evaluation Framework: TUM Trajectory I/O
// ─────────────────────────────────────────────────────────────────────────────
//
// Read/write trajectories in TUM RGB-D benchmark format:
//   timestamp tx ty tz qx qy qz qw
//
// Note: TUM uses (qx, qy, qz, qw) order, while SDK uses (qw, qx, qy, qz).
// The reader/writer handles the conversion automatically.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "dataset_adapter.h"
#include "pose_recorder.h"

#include <string>
#include <vector>

namespace eval {

namespace tum {

/// Write estimated poses in TUM format.
bool write(const std::vector<StampedPose>& poses, const std::string& path);

/// Write ground truth poses in TUM format.
bool write(const std::vector<GtPose>& poses, const std::string& path);

/// Read poses from TUM file (returns as GtPose since format is identical).
std::vector<GtPose> read(const std::string& path);

} // namespace tum
} // namespace eval
