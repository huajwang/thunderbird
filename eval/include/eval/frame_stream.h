// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Evaluation Framework: Frame Stream
// ─────────────────────────────────────────────────────────────────────────────
//
// Drives AcmeSlamEngine from a DatasetAdapter in strict timestamp order.
// No wall-clock sleeping — pure data-driven replay for determinism.
// Instruments every phase with PerfTimer hooks.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "dataset_adapter.h"
#include "metrics.h"
#include "perf_timer.h"

#include "thunderbird/odom/slam_engine.h"

#include <functional>
#include <vector>

namespace eval {

class FrameStream {
public:
    struct Stats {
        size_t imu_fed{0};
        size_t lidar_fed{0};
        double wall_time_s{0};
        double replay_fps{0};            // lidar frames / wall second
        std::vector<FrameTiming> timings; // per-frame timing records
    };

    /// Feed entire sequence into the engine.  Blocks until EOF or max_frames.
    ///
    /// @param adapter   Dataset reader (must be open()).
    /// @param engine    Initialised AcmeSlamEngine.
    /// @param progress  Optional callback after every LiDAR frame.
    ///                  Args: (frames_fed, total_expected).
    Stats replay(DatasetAdapter& adapter,
                 AcmeSlamEngine& engine,
                 std::function<void(size_t, size_t)> progress = nullptr);
};

} // namespace eval
