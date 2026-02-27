// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — 3D Object Detector abstract interface
// ─────────────────────────────────────────────────────────────────────────────
//
// Pure virtual base class for 3D point cloud object detectors.  Concrete
// implementations:
//
//   • CpuClusterDetector   — geometry-based cluster classifier (always available)
//   • GpuPillarDetector    — PointPillars via TensorRT  (requires GPU build)
//   • GpuCenterPointDetector — CenterPoint via TensorRT (requires GPU build)
//
// The factory method ObjectDetector::create() selects the correct backend
// at runtime based on PerceptionConfig::detector::backend and build flags.
//
// Thread safety:
//   • detect() is called from a SINGLE dedicated thread (T2).
//   • It is NOT thread-safe — do not call from multiple threads.
//   • initialize() must complete before the first detect() call.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/perception/perception_config.h"
#include "thunderbird/perception/perception_types.h"
#include "thunderbird/odom/slam_types.h"

#include <memory>
#include <vector>

namespace thunderbird::perception {

// ═════════════════════════════════════════════════════════════════════════════
//  DetectionInput — preprocessed data passed to the detector
// ═════════════════════════════════════════════════════════════════════════════
//
// Produced by PointCloudPreprocessor, consumed by ObjectDetector::detect().
// Contains the ground-removed, clustered point cloud along with metadata.
// ─────────────────────────────────────────────────────────────────────────────

struct DetectionInput {
    int64_t                  timestamp_ns{0};
    odom::Pose6D             ego_pose;

    /// Ground-removed, ROI-cropped point cloud.
    std::shared_ptr<const odom::PointCloudFrame> filtered_cloud;

    /// Per-point cluster label [point_idx → cluster_id].
    /// cluster_id == 0 means unassigned / noise.
    std::vector<uint32_t>    cluster_labels;

    /// Number of clusters (excluding label 0).
    uint32_t                 num_clusters{0};

    /// Ground plane coefficients (ax + by + cz + d = 0),
    /// filled by RANSAC mode; zeroed in height-threshold mode.
    double                   ground_plane[4]{};

    /// Number of raw points before preprocessing.
    uint32_t                 raw_point_count{0};
};

// ═════════════════════════════════════════════════════════════════════════════
//  ObjectDetector — abstract interface
// ═════════════════════════════════════════════════════════════════════════════

class ObjectDetector {
public:
    virtual ~ObjectDetector() = default;

    // ── Lifecycle ───────────────────────────────────────────────────────

    /// One-time initialisation.
    ///
    /// For GPU backends: loads TensorRT engine, allocates device memory,
    /// creates CUDA stream.  Blocking; may take 1–30 s for engine build.
    ///
    /// @param config  Full perception config (detector reads its subset).
    /// @return true on success.
    virtual bool initialize(const PerceptionConfig& config) = 0;

    /// Release all resources (GPU memory, model handles, etc.).
    /// Called automatically by the destructor.
    virtual void teardown() = 0;

    // ── Inference ───────────────────────────────────────────────────────

    /// Run synchronous detection on the calling thread.
    ///
    /// CONTRACT:
    ///   • Called from a single thread (T2 detector thread).
    ///   • input.filtered_cloud is non-null and deskewed.
    ///   • Returns a DetectionFrame with ego-frame bounding boxes.
    ///
    /// @param input  Preprocessed detection input.
    /// @return DetectionFrame with all detections above the confidence
    ///         threshold, NMS-filtered.
    virtual DetectionFrame detect(const DetectionInput& input) = 0;

    // ── Identity ────────────────────────────────────────────────────────

    /// Human-readable backend name (for logging / diagnostics).
    [[nodiscard]] virtual const char* name() const noexcept = 0;

    /// Whether this backend uses GPU acceleration.
    [[nodiscard]] virtual bool uses_gpu() const noexcept = 0;

    // ── Factory ─────────────────────────────────────────────────────────

    /// Create the correct detector for the given configuration.
    ///
    /// Selection logic:
    ///   1. If config requests GpuCenterPoint and THUNDERBIRD_HAS_GPU_PERCEPTION:
    ///        → GpuCenterPointDetector
    ///   2. If config requests GpuPointPillars and THUNDERBIRD_HAS_GPU_PERCEPTION:
    ///        → GpuPillarDetector
    ///   3. Otherwise:
    ///        → CpuClusterDetector (always available)
    ///
    /// @param config  Full perception config.
    /// @return Non-null unique_ptr to a detector instance.
    static std::unique_ptr<ObjectDetector> create(const PerceptionConfig& config);
};

} // namespace thunderbird::perception
