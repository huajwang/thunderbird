// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — PerceptionEngine: 3D detection & tracking pipeline
// ─────────────────────────────────────────────────────────────────────────────
//
// Top-level orchestrator for the 3D perception subsystem.  Consumes
// SlamOutput from AcmeSlamEngine and produces TrackedObjectList.
//
// ═══════════════════════════════════════════════════════════════════════════
//  Thread Model — 3 internal threads, 4 lock-free queues
// ═══════════════════════════════════════════════════════════════════════════
//
//  ┌────────────────────────────────────────────────────────────────────────┐
//  │                                                                        │
//  │  SLAM thread (external)              PerceptionEngine threads          │
//  │  ──────────────────────              ────────────────────────          │
//  │                                                                        │
//  │  feedSlamOutput()                                                      │
//  │     │                                                                  │
//  │     ▼                                                                  │
//  │  ┌──────────────────────┐    ┌──────────────────────────────────┐     │
//  │  │  perception_ring_    │    │  [T1] Preprocessor (CPU)          │     │
//  │  │  SPSC<SlamOutput, 16>│───►│  voxel + ground + cluster        │     │
//  │  └──────────────────────┘    └──────────────┬───────────────────┘     │
//  │                                              │                         │
//  │                              ┌───────────────▼──────────────────┐     │
//  │                              │  detection_ring_                  │     │
//  │                              │  SPSC<DetectionInput, 8>          │     │
//  │                              └───────────────┬──────────────────┘     │
//  │                                              │                         │
//  │                              ┌───────────────▼──────────────────┐     │
//  │                              │  [T2] Detector (GPU / CPU)        │     │
//  │                              │  PointPillars / CenterPoint /     │     │
//  │                              │  CpuCluster                       │     │
//  │                              └───────────────┬──────────────────┘     │
//  │                                              │                         │
//  │                              ┌───────────────▼──────────────────┐     │
//  │                              │  tracking_ring_                   │     │
//  │                              │  SPSC<DetectionFrame, 16>         │     │
//  │                              └───────────────┬──────────────────┘     │
//  │                                              │                         │
//  │                              ┌───────────────▼──────────────────┐     │
//  │                              │  [T3] Tracker (CPU)               │     │
//  │                              │  Hungarian + Kalman + lifecycle    │     │
//  │                              └───────────────┬──────────────────┘     │
//  │                                              │                         │
//  │  getDetectedObjects()        ┌───────────────▼──────────────────┐     │
//  │     │                        │  output_ring_                     │     │
//  │     ▼                        │  SPSC<TrackedObjectList*, 32>     │     │
//  │  ◄───────────────────────────│                                   │     │
//  │                              └──────────────────────────────────┘     │
//  │                                                                        │
//  └────────────────────────────────────────────────────────────────────────┘
//
// Thread safety contract:
//
//   • feedSlamOutput()      — single-producer (SLAM callback).     Non-blocking.
//   • getDetectedObjects()  — single-consumer (user thread).       Non-blocking.
//   • getLatestDetections() — single-consumer (user thread).       Non-blocking.
//   • initialize()          — call once before start().            Blocking.
//   • start() / stop()      — lifecycle, call from main thread.    Blocking.
//   • shutdown()            — call once; joins all threads.        Blocking.
//
// Non-blocking guarantee:
//
//   feedSlamOutput() performs a single lock-free SPSC push into
//   perception_ring_.  The SLAM worker thread is NEVER blocked by
//   perception processing.  If perception falls behind, frames are
//   dropped (ring overwrites oldest unprocessed entry).
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/perception/perception_config.h"
#include "thunderbird/perception/perception_types.h"
#include "thunderbird/odom/slam_types.h"

#include <cstdint>
#include <functional>
#include <memory>

namespace thunderbird::perception {

// ═════════════════════════════════════════════════════════════════════════════
//  PerceptionEngine — public API
// ═════════════════════════════════════════════════════════════════════════════
//
// Lifecycle:
//
//   PerceptionEngine engine;
//   engine.initialize(config);             // loads model, allocates GPU mem
//
//   // Wire to SLAM:
//   slam_engine.onSlamOutput([&](auto out) {
//       engine.feedSlamOutput(out);         // non-blocking, lock-free
//   });
//
//   engine.start();                         // spawns T1, T2, T3
//
//   // From user thread:
//   std::shared_ptr<const TrackedObjectList> objs;
//   if (engine.getDetectedObjects(objs))    // non-blocking pull
//       processObjects(*objs);
//
//   engine.stop();                          // joins threads
//   engine.shutdown();                      // releases resources
//
// ─────────────────────────────────────────────────────────────────────────────

class PerceptionEngine {
public:
    // ── Construction / Destruction ──────────────────────────────────────

    PerceptionEngine();
    ~PerceptionEngine();

    // Non-copyable, non-movable (owns threads + PImpl).
    PerceptionEngine(const PerceptionEngine&)            = delete;
    PerceptionEngine& operator=(const PerceptionEngine&) = delete;
    PerceptionEngine(PerceptionEngine&&)                 = delete;
    PerceptionEngine& operator=(PerceptionEngine&&)      = delete;

    // ── Lifecycle ───────────────────────────────────────────────────────

    /// One-time initialisation.
    ///
    /// Creates the preprocessor, loads the detector model (may take
    /// seconds for TensorRT engine build), and sets up ring buffers.
    ///
    /// @param config  Full perception configuration.
    /// @return true on success, false if detector failed to initialise.
    bool initialize(const PerceptionConfig& config);

    /// Spawn preprocessor (T1), detector (T2), and tracker (T3) threads.
    /// initialize() must have succeeded before calling start().
    void start();

    /// Signal threads to stop and join them.  Safe to call multiple times.
    void stop();

    /// Release all resources (model, GPU memory).  stop() is called
    /// implicitly if threads are still running.
    void shutdown();

    /// Is the pipeline currently running?
    [[nodiscard]] bool isRunning() const noexcept;

    // ── Data Ingress (non-blocking, lock-free) ──────────────────────────

    /// Feed a SLAM output into the perception pipeline.
    ///
    /// Called from the SLAM worker's onSlamOutput callback.  This is a
    /// single lock-free SPSC push (~100 ns).  If the ring is full,
    /// the oldest unprocessed frame is silently dropped.
    ///
    /// @param output  SLAM output containing pose + deskewed cloud.
    void feedSlamOutput(std::shared_ptr<const odom::SlamOutput> output);

    // ── Data Egress — Pull API (non-blocking, lock-free) ────────────────

    /// Retrieve the latest tracked object list.
    ///
    /// Returns the most recent output, consuming it from the ring.
    /// Multiple calls without new data will return false.
    ///
    /// @param[out] out  Receives a shared_ptr to the TrackedObjectList.
    /// @return true if a new result was available.
    bool getDetectedObjects(std::shared_ptr<const TrackedObjectList>& out);

    /// Retrieve the latest raw detection frame (before tracking).
    ///
    /// Only available if config.pipeline.publish_intermediate_detections
    /// is true.
    ///
    /// @param[out] out  Receives a shared_ptr to the DetectionFrame.
    /// @return true if a new result was available.
    bool getLatestDetections(std::shared_ptr<const DetectionFrame>& out);

    // ── Data Egress — Callback API ──────────────────────────────────────

    /// Register a callback for every TrackedObjectList output (~10 Hz).
    ///
    /// The callback fires on the tracker thread (T3) immediately after
    /// each tracking update.  It must not block.
    ///
    /// @param cb  Callback function.  Pass nullptr to unregister.
    void onTrackedObjects(TrackedObjectCallback cb);

    /// Register a callback for every raw DetectionFrame (~10 Hz).
    ///
    /// Fires on the detector thread (T2).  Must not block.
    ///
    /// @param cb  Callback function.  Pass nullptr to unregister.
    void onDetections(DetectionCallback cb);

    // ── Diagnostics ─────────────────────────────────────────────────────

    /// Cumulative pipeline statistics.
    struct Stats {
        uint64_t frames_received{0};     ///< SlamOutputs pushed into pipeline
        uint64_t frames_processed{0};    ///< frames through full pipeline
        uint64_t frames_dropped{0};      ///< dropped due to ring overflow
        double   avg_preprocess_ms{0};   ///< rolling average preprocess time
        double   avg_detection_ms{0};    ///< rolling average detection time
        double   avg_tracking_ms{0};     ///< rolling average tracking time
        double   avg_total_ms{0};        ///< rolling average end-to-end
        uint32_t active_tracks{0};       ///< currently confirmed tracks
        const char* detector_backend{""};///< active detector name
    };

    /// Query current pipeline statistics.
    ///
    /// Counter fields (frames_received, frames_processed, etc.) are atomic.
    /// Timing averages (avg_*_ms) are read from unsynchronised EMA accumulators
    /// and may be slightly stale or inconsistent — this is acceptable for
    /// diagnostics and avoids contention on the hot processing path.
    [[nodiscard]] Stats stats() const noexcept;

    /// Number of frames dropped at perception ingress ring.
    [[nodiscard]] size_t dropCount() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace thunderbird::perception
