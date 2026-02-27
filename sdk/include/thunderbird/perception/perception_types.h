// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — 3D Perception data types (public API)
// ─────────────────────────────────────────────────────────────────────────────
//
// These types form the public contract of the perception subsystem.  They are
// intentionally **Eigen-free** (same convention as slam_types.h) so the
// public header never pulls in heavy linear-algebra dependencies.
//
// Design principles (aligned with the SLAM type conventions):
//
//   • Timestamps are int64_t nanoseconds.
//   • Poses reference odom::Pose6D — no duplication.
//   • Shared ownership uses shared_ptr<const T> for zero-copy fan-out.
//   • All structs are standard-layout where possible.
//   • No CUDA, no TensorRT, no PCL in this header.
//
// Thread safety:
//   • All types are immutable after publication via shared_ptr<const T>.
//   • Pull API methods return shared_ptr copies (atomic ref-count bump).
//   • Callback delivery happens on an internal perception thread;
//     user callbacks must not block or modify the pointed-to data.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/odom/slam_types.h"

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace thunderbird::perception {

// ═════════════════════════════════════════════════════════════════════════════
//  Object class taxonomy
// ═════════════════════════════════════════════════════════════════════════════

/// Detected / tracked object class label.
///
/// The taxonomy is a union of car-mode and drone-mode classes.
/// Detectors only emit the subset relevant to their profile; the tracker
/// preserves whichever label the detector assigned.
enum class ObjectClass : uint8_t {
    Unknown       = 0,

    // ── Car-mode classes (KITTI / nuScenes origin) ──────────────────────
    Vehicle       = 1,
    Pedestrian    = 2,
    Cyclist       = 3,
    Barrier       = 4,

    // ── Drone-mode classes ──────────────────────────────────────────────
    Person        = 10,
    Pole          = 11,
    Wire          = 12,
    SmallVehicle  = 13,
};

/// Human-readable label for an ObjectClass value.
inline const char* object_class_name(ObjectClass c) noexcept {
    switch (c) {
        case ObjectClass::Unknown:      return "Unknown";
        case ObjectClass::Vehicle:      return "Vehicle";
        case ObjectClass::Pedestrian:   return "Pedestrian";
        case ObjectClass::Cyclist:      return "Cyclist";
        case ObjectClass::Barrier:      return "Barrier";
        case ObjectClass::Person:       return "Person";
        case ObjectClass::Pole:         return "Pole";
        case ObjectClass::Wire:         return "Wire";
        case ObjectClass::SmallVehicle: return "SmallVehicle";
    }
    return "Unknown";
}

// ═════════════════════════════════════════════════════════════════════════════
//  3D Oriented Bounding Box
// ═════════════════════════════════════════════════════════════════════════════

struct BBox3D {
    double center[3]{};      ///< x, y, z in the detection reference frame
    double extent[3]{};      ///< length, width, height (metres)
    double yaw{0.0};         ///< heading angle (radians, Z-up, counter-clockwise)
};

// ═════════════════════════════════════════════════════════════════════════════
//  Single Detection (per-frame, before tracking)
// ═════════════════════════════════════════════════════════════════════════════

struct Detection3D {
    BBox3D      bbox;            ///< oriented bounding box
    ObjectClass label{ObjectClass::Unknown};
    float       confidence{0};   ///< [0, 1]
    uint32_t    cluster_id{0};   ///< cross-reference to preprocessor cluster
    uint32_t    num_points{0};   ///< LiDAR points inside this detection
};

// ═════════════════════════════════════════════════════════════════════════════
//  Detection frame (output of detector, input to tracker)
// ═════════════════════════════════════════════════════════════════════════════

struct DetectionFrame {
    int64_t                   timestamp_ns{0};   ///< scan timestamp
    odom::Pose6D              ego_pose;           ///< ego pose at scan time
    std::vector<Detection3D>  detections;         ///< all detections this frame

    [[nodiscard]] uint32_t num_detections() const noexcept {
        return static_cast<uint32_t>(detections.size());
    }
};

// ═════════════════════════════════════════════════════════════════════════════
//  Track lifecycle state
// ═════════════════════════════════════════════════════════════════════════════

enum class TrackState : uint8_t {
    Tentative = 0,   ///< newly created, waiting for confirm_hits
    Confirmed = 1,   ///< stable, actively matched
    Coasting  = 2,   ///< missed detection, prediction-only
};

/// Human-readable name.
inline const char* track_state_name(TrackState s) noexcept {
    switch (s) {
        case TrackState::Tentative: return "Tentative";
        case TrackState::Confirmed: return "Confirmed";
        case TrackState::Coasting:  return "Coasting";
    }
    return "Unknown";
}

// ═════════════════════════════════════════════════════════════════════════════
//  Single Tracked Object
// ═════════════════════════════════════════════════════════════════════════════
//
// Published via shared_ptr<const TrackedObjectList>.  The track maintains
// filtered state (position, velocity, size) using a per-object Kalman filter.
//
// Covariance is the 7×7 block [x, y, z, yaw, vx, vy, vz] stored row-major.
// ─────────────────────────────────────────────────────────────────────────────

struct TrackedObject {
    uint64_t    track_id{0};            ///< unique, monotonic, never reused
    TrackState  state{TrackState::Tentative};
    ObjectClass label{ObjectClass::Unknown};
    float       confidence{0};          ///< smoothed confidence

    BBox3D      bbox;                    ///< filtered position + extent
    double      velocity[3]{};           ///< vx, vy, vz  (m/s, world frame)
    double      yaw_rate{0};             ///< rad/s

    int         age_frames{0};           ///< frames since first detection
    int         hits{0};                 ///< total matched detections
    int         consecutive_misses{0};   ///< current coast streak

    /// 7×7 covariance (x, y, z, yaw, vx, vy, vz) row-major
    double      covariance[49]{};
};

// ═════════════════════════════════════════════════════════════════════════════
//  Tracked Object List (published per frame)
// ═════════════════════════════════════════════════════════════════════════════

struct TrackedObjectList {
    int64_t                       timestamp_ns{0};
    odom::Pose6D                  ego_pose;
    std::vector<TrackedObject>    objects;
    uint32_t                      frame_sequence{0};

    // ── Per-frame diagnostics ───────────────────────────────────────────
    double   preprocess_ms{0};       ///< preprocessing wall-clock time
    double   detection_ms{0};        ///< detection wall-clock time
    double   tracking_ms{0};         ///< tracking wall-clock time
    uint32_t input_points{0};        ///< raw point count before filtering
    uint32_t filtered_points{0};     ///< point count after filtering
    uint32_t num_clusters{0};        ///< clusters from preprocessor
    uint32_t num_detections{0};      ///< detections before tracking

    [[nodiscard]] uint32_t num_objects() const noexcept {
        return static_cast<uint32_t>(objects.size());
    }

    [[nodiscard]] double total_latency_ms() const noexcept {
        return preprocess_ms + detection_ms + tracking_ms;
    }
};

// ═════════════════════════════════════════════════════════════════════════════
//  Callbacks
// ═════════════════════════════════════════════════════════════════════════════

/// Callback for tracked object output (~10 Hz, after tracker update).
using TrackedObjectCallback = std::function<void(std::shared_ptr<const TrackedObjectList>)>;

/// Callback for raw detection output (~10 Hz, before tracking).
using DetectionCallback     = std::function<void(std::shared_ptr<const DetectionFrame>)>;

} // namespace thunderbird::perception
