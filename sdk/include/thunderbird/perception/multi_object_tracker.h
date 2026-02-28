// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — 3D Multi-Object Tracker (public header)
// ─────────────────────────────────────────────────────────────────────────────
//
// Kalman-filter-based multi-object tracker with Hungarian (Munkres)
// assignment for optimal detection-to-track association.
//
//   • Per-object 7-state constant-velocity Kalman filter
//     State vector: [x, y, z, yaw, vx, vy, vz]
//   • O(n³) Hungarian algorithm for globally-optimal assignment
//   • Configurable association metric (IoU-3D, centre distance, Mahalanobis)
//   • Gating to reject implausible matches before assignment
//   • Track lifecycle:  Tentative → Confirmed → Coasting → Deleted
//   • Monotonic, never-reused track IDs
//   • Covariance published per-track for downstream fusion
//
// Thread safety:
//   This class is NOT thread-safe.  It is designed to be owned exclusively
//   by the T3 tracker thread inside PerceptionEngine::Impl.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/perception/perception_types.h"
#include "thunderbird/perception/perception_config.h"

#include <cstdint>
#include <memory>

namespace thunderbird::perception {

class MultiObjectTracker {
public:
    explicit MultiObjectTracker(const TrackerConfig& config);
    ~MultiObjectTracker();

    // Non-copyable, movable.
    MultiObjectTracker(const MultiObjectTracker&)            = delete;
    MultiObjectTracker& operator=(const MultiObjectTracker&) = delete;
    MultiObjectTracker(MultiObjectTracker&&) noexcept;
    MultiObjectTracker& operator=(MultiObjectTracker&&) noexcept;

    /// Run one tracking cycle: predict → associate → update → manage.
    ///
    /// @param  detections  Current-frame detections from the detector.
    /// @return Tracked object list with only Confirmed + Coasting tracks.
    TrackedObjectList update(const DetectionFrame& detections);

    /// Reset all state (clears all tracks, resets ID counter).
    void reset();

    /// Number of currently active tracks (all states).
    [[nodiscard]] uint32_t activeTrackCount() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace thunderbird::perception
