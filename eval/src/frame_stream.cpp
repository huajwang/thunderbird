// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Evaluation Framework: Frame Stream Implementation
// ─────────────────────────────────────────────────────────────────────────────

#include "eval/frame_stream.h"
#include "eval/perf_timer.h"

#include <chrono>
#include <iostream>
#include <thread>
#include <variant>

namespace eval {

FrameStream::Stats FrameStream::replay(
    DatasetAdapter& adapter,
    AcmeSlamEngine& engine,
    std::function<void(size_t, size_t)> progress)
{
    Stats stats;
    PerfTimer timer;
    ReplayRateLogger rate_logger(5.0);  // log every 5 s

    const auto di = adapter.info();
    const auto wall_start = PerfTimer::now();

    rate_logger.start();

    size_t imu_batch = 0;  // IMU samples since last LiDAR frame
    FrameTiming current_timing;
    bool   in_frame = false;
    PerfTimer::TimePoint frame_wall_start;

    while (auto ev = adapter.next()) {
        if (std::holds_alternative<ImuSample>(ev->payload)) {
            // ── IMU event ───────────────────────────────────────────────
            timer.start("feed_imu");
            engine.feedImu(std::get<ImuSample>(ev->payload));
            timer.stop_ms("feed_imu");

            stats.imu_fed++;
            imu_batch++;

        } else {
            // ── LiDAR event ─────────────────────────────────────────────
            auto cloud = std::get<std::shared_ptr<const PointCloudFrame>>(ev->payload);

            // If we were accumulating a frame, finalise its timing.
            if (in_frame) {
                // Wait briefly for engine to produce output.
                timer.start("wait");
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                current_timing.wait_ms = timer.stop_ms("wait");
                current_timing.wait_ns = timer.last_stop_ns();
                current_timing.total_ms = current_timing.load_ms +
                                          current_timing.feed_ms +
                                          current_timing.wait_ms;
                current_timing.total_ns = PerfTimer::elapsed_since_ns(frame_wall_start);
                stats.timings.push_back(current_timing);
            }

            // Start new frame timing.
            current_timing = {};
            current_timing.frame_index  = stats.lidar_fed;
            current_timing.timestamp_ns = ev->timestamp_ns;
            in_frame = true;
            frame_wall_start = PerfTimer::now();

            // The load time is tracked inside the adapter (PerfTimer "load_bin").
            // We track feed time here.
            timer.start("feed_cloud");
            engine.feedPointCloud(std::move(cloud));
            current_timing.feed_ms = timer.stop_ms("feed_cloud");
            current_timing.feed_ns = timer.last_stop_ns();

            stats.lidar_fed++;
            imu_batch = 0;

            // Rate logging.
            rate_logger.tick(stats.lidar_fed, 0);

            // Progress callback.
            if (progress) {
                progress(stats.lidar_fed, 0);
            }
        }
    }

    // Finalise last frame.
    if (in_frame) {
        timer.start("wait");
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        current_timing.wait_ms = timer.stop_ms("wait");
        current_timing.wait_ns = timer.last_stop_ns();
        current_timing.total_ms = current_timing.load_ms +
                                  current_timing.feed_ms +
                                  current_timing.wait_ms;
        current_timing.total_ns = PerfTimer::elapsed_since_ns(frame_wall_start);
        stats.timings.push_back(current_timing);
    }

    // Wall time.
    stats.wall_time_s = PerfTimer::elapsed_since_ms(wall_start) / 1000.0;
    stats.replay_fps  = stats.lidar_fed > 0
        ? static_cast<double>(stats.lidar_fed) / stats.wall_time_s
        : 0.0;

    rate_logger.finish();

    return stats;
}

} // namespace eval
