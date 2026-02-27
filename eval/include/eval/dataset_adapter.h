// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Evaluation Framework: Dataset Adapter Interface
// ─────────────────────────────────────────────────────────────────────────────
//
// Abstract streaming reader for dataset sequences (KITTI, EuRoC, .tbrec).
// Emits timestamped events (IMU or LiDAR) in strict order, one at a time,
// so the entire dataset is never held in memory.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/odom/slam_types.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace eval {

// Forward declaration.
struct ImuInterpolatorConfig;

using namespace thunderbird::odom;

// ═════════════════════════════════════════════════════════════════════════════
//  StreamEvent — a single timestamped datum from the dataset
// ═════════════════════════════════════════════════════════════════════════════

struct StreamEvent {
    int64_t timestamp_ns{0};
    int64_t load_ns{0};      ///< Time spent loading this datum from disk (0 if N/A).
    std::variant<
        ImuSample,
        std::shared_ptr<const PointCloudFrame>
    > payload;
};

// ═════════════════════════════════════════════════════════════════════════════
//  GtPose — ground truth pose at a single timestamp
// ═════════════════════════════════════════════════════════════════════════════

struct GtPose {
    int64_t              timestamp_ns{0};
    std::array<double,3> position{};       // x y z  (metres, world frame)
    std::array<double,4> quaternion{1,0,0,0}; // w x y z (Hamilton, body→world)
};

// ═════════════════════════════════════════════════════════════════════════════
//  DatasetInfo — metadata about a sequence
// ═════════════════════════════════════════════════════════════════════════════

struct DatasetInfo {
    std::string name;       // e.g. "kitti_00", "euroc_MH_01"
    std::string format;     // "kitti" | "euroc" | "tbrec"
    double      duration_s{0};
    double      distance_m{0};  // total GT path length (0 if unknown)
    bool        has_lidar{false};
    bool        has_imu{false};
    bool        has_ground_truth{false};
};

// ═════════════════════════════════════════════════════════════════════════════
//  DatasetAdapter — abstract streaming reader
// ═════════════════════════════════════════════════════════════════════════════
//
// Contract:
//   - next() yields events in strict timestamp order.
//   - Streaming: at most one LiDAR scan in memory at a time.
//   - Deterministic: same sequence → same output, always.
//   - Thread safety: single-threaded reader, no internal locking.

class DatasetAdapter {
public:
    virtual ~DatasetAdapter() = default;

    /// Open a dataset sequence at `path`.
    /// @param max_frames  Maximum LiDAR frames to emit (0 = unlimited).
    /// @return false if the path is invalid or format unrecognised.
    virtual bool open(const std::string& path, size_t max_frames = 0) = 0;

    /// Dataset metadata (valid after open()).
    virtual DatasetInfo info() const = 0;

    /// Next event in timestamp order.  Returns std::nullopt at EOF.
    virtual std::optional<StreamEvent> next() = 0;

    /// Load full ground truth trajectory (called once before replay).
    /// Returns empty vector if no GT available.
    virtual std::vector<GtPose> loadGroundTruth() = 0;

    /// Reset to beginning of sequence (for re-runs).
    virtual void rewind() = 0;

    /// Total LiDAR frames emitted so far.
    virtual size_t framesEmitted() const = 0;
};

// ═════════════════════════════════════════════════════════════════════════════
//  Factory functions
// ═════════════════════════════════════════════════════════════════════════════

/// Create adapter by explicit format string ("kitti", "euroc", "tbrec").
std::unique_ptr<DatasetAdapter> createAdapter(const std::string& format);

/// Create adapter with IMU interpolation options.
std::unique_ptr<DatasetAdapter> createAdapter(const std::string& format,
                                               bool enable_imu,
                                               ImuInterpolatorConfig imu_cfg);

/// Create adapter by path heuristic:
///   - *.tbrec → TbrecAdapter
///   - dir containing velodyne/ → KittiAdapter
///   - dir containing imu0/    → EurocAdapter
std::unique_ptr<DatasetAdapter> createAdapterFromPath(const std::string& path);

/// Create adapter from path with IMU interpolation options.
std::unique_ptr<DatasetAdapter> createAdapterFromPath(const std::string& path,
                                                       bool enable_imu,
                                                       ImuInterpolatorConfig imu_cfg);

} // namespace eval
