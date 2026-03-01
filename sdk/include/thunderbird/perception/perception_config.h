// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Perception pipeline configuration
// ─────────────────────────────────────────────────────────────────────────────
//
// All tunable parameters for the perception subsystem:
//   • Preprocessor (voxel filter, ground removal, clustering)
//   • Detector (backend selection, model path, thresholds)
//   • Tracker (motion model, association, lifecycle)
//   • Pipeline (threading, rate control, ring buffer sizing)
//
// These map 1:1 to the `perception:` section and subsections in the
// platform YAML configs (drone.yaml, car.yaml).
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace thunderbird::perception {

// ═════════════════════════════════════════════════════════════════════════════
//  Preprocessor configuration
// ═════════════════════════════════════════════════════════════════════════════

/// Ground removal strategy.
enum class GroundRemovalMethod : uint8_t {
    HeightThreshold = 0,   ///< fast: remove points below fixed z
    RansacPlane     = 1,   ///< robust: fit plane then remove inliers
};

struct PreprocessorConfig {
    double voxel_size{0.15};               ///< voxel edge length for downsampling (m)
    double roi_radius{50.0};               ///< ROI bounding-sphere radius (m)
    double roi_z_min{-5.0};                ///< ROI z-axis minimum (m)
    double roi_z_max{5.0};                 ///< ROI z-axis maximum (m)

    GroundRemovalMethod ground_method{GroundRemovalMethod::RansacPlane};
    double ground_height_threshold{-0.3};  ///< z-threshold for HeightThreshold mode (m)
    double ransac_distance_threshold{0.15};///< inlier distance for RANSAC plane (m)
    int    ransac_max_iterations{100};     ///< RANSAC iterations

    double cluster_eps{0.6};               ///< Euclidean clustering radius (m)
    int    cluster_min_points{10};          ///< minimum points per cluster
    int    cluster_max_points{50'000};      ///< maximum points per cluster
    int    max_clusters{256};               ///< hard limit on returned clusters
};

// ═════════════════════════════════════════════════════════════════════════════
//  Detector configuration
// ═════════════════════════════════════════════════════════════════════════════

/// Detector backend selection.
///
/// The factory ObjectDetector::create() uses this to instantiate the
/// correct implementation.  GPU backends require THUNDERBIRD_HAS_GPU_PERCEPTION
/// to be defined at build time; otherwise they fall through to CpuCluster.
enum class DetectorBackend : uint8_t {
    CpuCluster     = 0,   ///< geometry-based cluster classifier (no GPU)
    GpuPointPillars= 1,   ///< PointPillars via TensorRT FP16
    GpuCenterPoint = 2,   ///< CenterPoint  via TensorRT FP16
};

struct DetectorConfig {
    DetectorBackend backend{DetectorBackend::CpuCluster};

    // ── GPU model paths (ignored for CpuCluster) ────────────────────────
    std::string model_path;                ///< path to TensorRT .engine file
    std::string model_config_path;         ///< optional: model-specific YAML

    // ── Inference thresholds ────────────────────────────────────────────
    float  confidence_threshold{0.35f};    ///< discard detections below this
    float  nms_iou_threshold{0.5f};        ///< NMS IoU overlap threshold
    int    max_detections{200};            ///< cap per frame

    // ── PointPillars-specific ───────────────────────────────────────────
    double pillar_x_size{0.16};            ///< pillar grid resolution, X (m)
    double pillar_y_size{0.16};            ///< pillar grid resolution, Y (m)
    int    max_pillars{12'000};            ///< max occupied pillars per frame
    int    max_points_per_pillar{32};      ///< max points per pillar

    // ── Point cloud range for voxelization ──────────────────────────────
    double pc_range_x_min{0.0};
    double pc_range_x_max{69.12};
    double pc_range_y_min{-39.68};
    double pc_range_y_max{39.68};
    double pc_range_z_min{-3.0};
    double pc_range_z_max{1.0};
};

// ═════════════════════════════════════════════════════════════════════════════
//  Tracker configuration
// ═════════════════════════════════════════════════════════════════════════════

/// Per-object motion model selection.
enum class MotionModel : uint8_t {
    ConstantVelocity = 0,   ///< linear CV model (drone default)
    CTRV             = 1,   ///< constant turn-rate and velocity (car default)
                            ///< **NOT YET IMPLEMENTED** — currently falls back
                            ///< to ConstantVelocity.  Reserved for future use.
};

/// Association metric for detection → track matching.
enum class AssociationMetric : uint8_t {
    IoU3D           = 0,   ///< 3D intersection-over-union
    CenterDistance  = 1,   ///< Euclidean distance between centres
    Mahalanobis     = 2,   ///< Mahalanobis distance using track covariance
};

struct TrackerConfig {
    MotionModel       motion_model{MotionModel::ConstantVelocity};
    AssociationMetric association_metric{AssociationMetric::IoU3D};

    int    confirm_hits{3};          ///< hits before Tentative → Confirmed
    int    max_coast_frames{5};      ///< misses before Confirmed → Deleted
    int    tentative_max_misses{2};  ///< misses before Tentative → Deleted

    // ── Kalman filter noise ────────────────────────────────────────────
    double process_noise_pos{0.5};   ///< position process noise σ (m)
    double process_noise_vel{1.0};   ///< velocity process noise σ (m/s)
    double process_noise_yaw{0.1};   ///< yaw process noise σ (rad)
    double measurement_noise{0.3};   ///< measurement noise σ (m)

    // ── Gating ──────────────────────────────────────────────────────────
    double association_gate{5.0};    ///< gating distance (metric-dependent)
};

// ═════════════════════════════════════════════════════════════════════════════
//  Pipeline configuration
// ═════════════════════════════════════════════════════════════════════════════

struct PipelineConfig {
    // ── Rate control ────────────────────────────────────────────────────
    /// Maximum inference rate (Hz).  If SLAM outputs arrive faster, the
    /// pipeline will skip intermediate frames.  0 = process every frame.
    double max_inference_rate_hz{0.0};

    // ── Ring buffer capacities (power-of-two) ──────────────────────────
    static constexpr size_t kPerceptionRingCapacity = 16;
    static constexpr size_t kDetectionRingCapacity  = 8;
    static constexpr size_t kTrackingRingCapacity   = 16;
    static constexpr size_t kOutputRingCapacity     = 32;

    // ── GPU device ──────────────────────────────────────────────────────
    int cuda_device_id{0};             ///< CUDA device ordinal

    // ── Diagnostics ─────────────────────────────────────────────────────
    bool publish_intermediate_detections{false}; ///< expose raw DetectionFrame
};

// ═════════════════════════════════════════════════════════════════════════════
//  Master perception configuration
// ═════════════════════════════════════════════════════════════════════════════

struct PerceptionConfig {
    bool               enable{true};

    PreprocessorConfig preprocessor;
    DetectorConfig     detector;
    TrackerConfig      tracker;
    PipelineConfig     pipeline;
};

} // namespace thunderbird::perception
