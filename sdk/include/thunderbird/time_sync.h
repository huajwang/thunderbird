// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Time Synchronization Layer (Step 3)
// ─────────────────────────────────────────────────────────────────────────────
//
// Aligns LiDAR, Camera, and IMU streams into a single `SyncedFrame` using
// hardware timestamps.  The synchronization policy is:
//
//   • **LiDAR-driven**: each LiDAR frame defines a sync epoch.
//   • **Camera**: nearest-neighbour match within a configurable tolerance.
//   • **IMU block**: *all* IMU samples whose timestamps fall between the
//     previous LiDAR epoch and the current one are collected into a vector.
//     This is critical for downstream motion compensation / state estimation.
//
// ┌──────────┐   ┌──────────┐   ┌──────────┐
// │  LiDAR   │   │   IMU    │   │  Camera  │
// │ (10 Hz)  │   │ (200 Hz) │   │ (30 Hz)  │
// └────┬─────┘   └────┬─────┘   └────┬─────┘
//      │               │              │
//      │   feed()      │  feed()      │  feed()
//      ▼               ▼              ▼
//  ┌─────────────────────────────────────────┐
//  │            TimeSyncEngine               │
//  │                                         │
//  │  lidar_q_   imu_q_   camera_q_          │
//  │      │         │         │              │
//  │      └─────────┼─────────┘              │
//  │         try_assemble()                  │
//  │              │                          │
//  │     ┌────────▼────────┐                 │
//  │     │   SyncedFrame   │──► callback     │
//  │     └────────┬────────┘                 │
//  │              │                          │
//  │         sync_queue_  ──────► pull API   │
//  └─────────────────────────────────────────┘
//
// Design decisions:
//   • **LiDAR as reference**: LiDAR is the lowest-rate sensor (10 Hz) and
//     each sweep is the natural unit of work for most robotics/perception
//     pipelines.  Camera & IMU align *to* it.
//   • **IMU block, not single sample**: downstream EKFs / pre-integration
//     need the full IMU trace between keyframes, not just the nearest one.
//   • **Tolerance window**: a configurable `camera_tolerance_ns` rejects
//     camera frames that are too far from the LiDAR epoch (latency spike,
//     drop, etc.).  The `SyncedFrame::camera` is left empty in that case.
//   • **Drift tracking**: a simple linear-regression model over the last N
//     LiDAR↔Camera offset pairs predicts the expected offset.  If the
//     measured offset exceeds `drift_warn_ns` the engine emits a
//     diagnostic callback so the application can log / alert.
//   • **Thread safety**: `feed*()` can be called concurrently from
//     different producer threads; the engine's internal mutex serialises
//     access.  The sync thread runs on its own cadence.
//   • **Dual API (pull + callback)**: same pattern as DataLayer.
//   • **ROS2 / Python ready**: `SyncedFrame` is a plain-old-data struct
//     that can be trivially wrapped in a pybind11 binding or a ROS2 msg.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/sensor_data.h"
#include "thunderbird/sensor_queue.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <numeric>
#include <optional>
#include <thread>
#include <vector>

namespace thunderbird::data {

// ─── Synced output ──────────────────────────────────────────────────────────

/// A fully-synchronized multi-sensor snapshot.
///
///   • `lidar`     — the reference LiDAR sweep (always present).
///   • `camera`    — the nearest camera frame within tolerance, or empty
///                   if no camera frame was close enough.
///   • `imu_block` — all IMU samples between the *previous* LiDAR epoch
///                   and this one (may be empty on the first frame).
///   • `sync_quality` — 0.0 (poor) to 1.0 (perfect) heuristic.
struct SyncedFrame {
    LidarFrame              lidar;           ///< reference sweep
    std::optional<ImageFrame> camera;        ///< nearest camera (may be empty)
    std::vector<ImuFrame>   imu_block;       ///< IMU between prev & this epoch

    int64_t  lidar_camera_offset_ns{0};      ///< camera_ts − lidar_ts (signed)
    float    sync_quality{0.0f};             ///< 0=poor, 1=perfect
    uint64_t sequence{0};                    ///< monotonic bundle counter
};

// ─── Configuration ──────────────────────────────────────────────────────────

struct TimeSyncConfig {
    /// Max allowable |lidar_ts − camera_ts|.  Camera frames outside this
    /// window are treated as missing and the SyncedFrame ships without one.
    int64_t camera_tolerance_ns = 50'000'000;   // 50 ms

    /// How often the sync thread wakes up to check for new bundles.
    int poll_interval_ms = 2;

    /// Number of recent offset samples used for drift estimation.
    size_t drift_window = 50;

    /// If the estimated drift rate exceeds this many ns/s, fire the
    /// drift warning callback so apps can log / alert.
    int64_t drift_warn_ns_per_sec = 1'000'000;   // 1 ms/s

    /// Max queue depth for each sensor inside the sync engine.
    size_t max_queue_depth = 128;

    /// Output queue depth for assembled SyncedFrames (pull API).
    size_t output_queue_depth = 32;
};

// ─── Callbacks ──────────────────────────────────────────────────────────────

using SyncedFrameCallback = std::function<void(const SyncedFrame&)>;

/// Diagnostic callback: reports estimated clock drift in ns/s.
using DriftCallback = std::function<void(double drift_ns_per_sec)>;

// ─── Sync statistics ────────────────────────────────────────────────────────

struct SyncStats {
    uint64_t frames_produced{0};       ///< total SyncedFrames emitted
    uint64_t camera_misses{0};         ///< LiDAR frames with no camera match
    uint64_t imu_gaps{0};              ///< LiDAR frames with empty IMU block
    double   mean_offset_ns{0};        ///< running mean lidar↔camera offset
    double   drift_ns_per_sec{0};      ///< latest drift estimate
};

// ─────────────────────────────────────────────────────────────────────────────

class TimeSyncEngine {
public:
    explicit TimeSyncEngine(TimeSyncConfig config = {})
        : config_(config)
        , output_q_(config.output_queue_depth) {}

    ~TimeSyncEngine() { stop(); }

    // ── Feed (producer side) ────────────────────────────────────────────

    /// Feed a converted LiDAR frame (from DataLayer or directly).
    void feedLidar(const LidarFrame& f) {
        std::lock_guard<std::mutex> lk(mu_);
        lidar_q_.push_back(f);
        trim(lidar_q_);
    }

    /// Feed a converted IMU frame.
    void feedImu(const ImuFrame& f) {
        std::lock_guard<std::mutex> lk(mu_);
        imu_q_.push_back(f);
        trim(imu_q_);
    }

    /// Feed a converted camera frame.
    void feedCamera(const ImageFrame& f) {
        std::lock_guard<std::mutex> lk(mu_);
        camera_q_.push_back(f);
        trim(camera_q_);
    }

    // ── Pull API ────────────────────────────────────────────────────────

    /// Non-blocking: returns std::nullopt when no SyncedFrame is ready.
    [[nodiscard]] std::optional<SyncedFrame> getNextSyncedFrame() {
        return output_q_.try_pop();
    }

    /// Blocking pull with timeout.
    template <typename Rep, typename Period>
    [[nodiscard]] std::optional<SyncedFrame> waitForSyncedFrame(
        std::chrono::duration<Rep, Period> timeout)
    {
        return output_q_.pop_for(timeout);
    }

    // ── Callback API ────────────────────────────────────────────────────

    void onSyncedFrame(SyncedFrameCallback cb) { sync_cb_ = std::move(cb); }
    void onDriftWarning(DriftCallback cb)       { drift_cb_ = std::move(cb); }

    // ── Lifecycle ───────────────────────────────────────────────────────

    void start() {
        if (running_.exchange(true)) return;
        thread_ = std::thread([this] { run(); });
    }

    void stop() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

    bool running() const { return running_.load(); }

    // ── Statistics ──────────────────────────────────────────────────────

    SyncStats stats() const {
        std::lock_guard<std::mutex> lk(mu_);
        return stats_;
    }

    /// Current count of assembled frames (lock-free).
    uint64_t framesProduced() const {
        return frames_produced_.load(std::memory_order_relaxed);
    }

    /// Drain all internal queues + output queue.
    void flush() {
        std::lock_guard<std::mutex> lk(mu_);
        lidar_q_.clear();
        imu_q_.clear();
        camera_q_.clear();
        output_q_.clear();
        prev_lidar_ts_ = std::numeric_limits<int64_t>::min();
    }

private:
    // ── Queue management ────────────────────────────────────────────────

    template <typename Deque>
    void trim(Deque& dq) const {
        while (dq.size() > config_.max_queue_depth) dq.pop_front();
    }

    // ── Main loop ───────────────────────────────────────────────────────

    void run() {
        while (running_) {
            {
                std::lock_guard<std::mutex> lk(mu_);
                while (try_assemble()) { /* consume all ready bundles */ }
            }
            std::this_thread::sleep_for(
                std::chrono::milliseconds(config_.poll_interval_ms));
        }
        // Final drain after stop
        std::lock_guard<std::mutex> lk(mu_);
        while (try_assemble()) {}
    }

    // ── Assembly ────────────────────────────────────────────────────────

    /// Try to build one SyncedFrame from the front of the queues.
    /// Must be called with mu_ held.  Returns true if a frame was emitted.
    bool try_assemble() {
        if (lidar_q_.empty()) return false;

        // The oldest unconsumed LiDAR frame is the reference.
        const LidarFrame& ref = lidar_q_.front();
        const int64_t ref_ts  = ref.timestamp_ns;

        // ── Camera: nearest-neighbour within tolerance ──────────────────
        std::optional<ImageFrame> best_cam;
        int64_t best_cam_diff = std::numeric_limits<int64_t>::max();

        for (size_t i = 0; i < camera_q_.size(); ++i) {
            const int64_t diff = std::abs(camera_q_[i].timestamp_ns - ref_ts);
            if (diff < best_cam_diff) {
                best_cam_diff = diff;
                best_cam      = camera_q_[i];
            }
        }

        if (best_cam && best_cam_diff > config_.camera_tolerance_ns) {
            best_cam.reset();  // outside tolerance — treat as missing
        }

        // ── IMU block: all samples in (prev_lidar_ts_, ref_ts] ──────────
        std::vector<ImuFrame> imu_block;
        while (!imu_q_.empty() && imu_q_.front().timestamp_ns <= ref_ts) {
            if (imu_q_.front().timestamp_ns > prev_lidar_ts_) {
                imu_block.push_back(imu_q_.front());
            }
            imu_q_.pop_front();
        }

        // ── Build SyncedFrame ───────────────────────────────────────────
        SyncedFrame sf;
        sf.lidar     = ref;
        sf.camera    = std::move(best_cam);
        sf.imu_block = std::move(imu_block);
        sf.sequence  = frames_produced_.load(std::memory_order_relaxed);

        // Offset & quality
        if (sf.camera) {
            sf.lidar_camera_offset_ns = sf.camera->timestamp_ns - ref_ts;
            // Quality: 1.0 at 0 offset, decays linearly to 0.0 at tolerance.
            const double ratio =
                static_cast<double>(std::abs(sf.lidar_camera_offset_ns)) /
                static_cast<double>(config_.camera_tolerance_ns);
            sf.sync_quality = static_cast<float>(
                std::clamp(1.0 - ratio, 0.0, 1.0));
        } else {
            sf.lidar_camera_offset_ns = 0;
            sf.sync_quality = 0.0f;
            ++stats_.camera_misses;
        }

        if (sf.imu_block.empty()) {
            ++stats_.imu_gaps;
        }

        // ── Drift estimation ────────────────────────────────────────────
        if (sf.camera) {
            update_drift(sf.lidar_camera_offset_ns, ref_ts);
        }

        // ── Evict consumed data ─────────────────────────────────────────
        lidar_q_.pop_front();

        // Evict camera frames older than the reference (no longer useful).
        if (best_cam) {
            // Remove up to and including the matched camera frame.
            while (!camera_q_.empty() &&
                   camera_q_.front().timestamp_ns <= best_cam->timestamp_ns) {
                camera_q_.pop_front();
            }
        } else {
            // No match — still evict camera frames that are too old.
            while (!camera_q_.empty() &&
                   camera_q_.front().timestamp_ns < ref_ts - config_.camera_tolerance_ns) {
                camera_q_.pop_front();
            }
        }

        prev_lidar_ts_ = ref_ts;

        // ── Update stats ────────────────────────────────────────────────
        ++stats_.frames_produced;
        if (!offset_history_.empty()) {
            double sum = 0;
            for (auto v : offset_history_) sum += v;
            stats_.mean_offset_ns = sum / static_cast<double>(offset_history_.size());
        }
        stats_.drift_ns_per_sec = drift_ns_per_sec_;

        frames_produced_.fetch_add(1, std::memory_order_relaxed);

        // ── Deliver ─────────────────────────────────────────────────────
        output_q_.push(sf);
        if (sync_cb_) sync_cb_(sf);

        return true;
    }

    // ── Drift tracking ──────────────────────────────────────────────────
    //
    // We maintain a sliding window of (lidar_ts, offset) pairs and run a
    // simple linear regression to estimate the drift rate (ns/s).  This
    // handles gradual clock divergence between the host and device.

    void update_drift(int64_t offset_ns, int64_t lidar_ts) {
        offset_history_.push_back(static_cast<double>(offset_ns));
        ts_history_.push_back(static_cast<double>(lidar_ts));

        while (offset_history_.size() > config_.drift_window) {
            offset_history_.pop_front();
            ts_history_.pop_front();
        }

        if (offset_history_.size() < 3) return; // need a few points

        // Simple OLS: slope = Σ((t-t̄)(o-ō)) / Σ((t-t̄)²)
        const size_t n = offset_history_.size();
        double t_mean = 0, o_mean = 0;
        for (size_t i = 0; i < n; ++i) {
            t_mean += ts_history_[i];
            o_mean += offset_history_[i];
        }
        t_mean /= static_cast<double>(n);
        o_mean /= static_cast<double>(n);

        double num = 0, den = 0;
        for (size_t i = 0; i < n; ++i) {
            const double dt = ts_history_[i] - t_mean;
            const double dv = offset_history_[i] - o_mean;
            num += dt * dv;
            den += dt * dt;
        }

        if (den < 1e-12) return; // degenerate — timestamps too close

        // slope is in (ns offset) / (ns time), i.e. dimensionless.
        // Convert to ns-per-second by multiplying by 1e9.
        const double slope = num / den;
        drift_ns_per_sec_  = slope * 1.0e9;

        // Fire warning if drift exceeds threshold.
        if (std::abs(drift_ns_per_sec_) >
            static_cast<double>(config_.drift_warn_ns_per_sec))
        {
            if (drift_cb_) drift_cb_(drift_ns_per_sec_);
        }
    }

    // ── State ───────────────────────────────────────────────────────────

    TimeSyncConfig config_;

    mutable std::mutex mu_;
    std::deque<LidarFrame>  lidar_q_;
    std::deque<ImuFrame>    imu_q_;
    std::deque<ImageFrame>  camera_q_;

    /// Timestamp of the previously consumed LiDAR frame.  IMU samples
    /// older than this have already been assigned to a prior bundle.
    int64_t prev_lidar_ts_{std::numeric_limits<int64_t>::min()};

    /// Drift estimation sliding windows.
    std::deque<double> offset_history_;
    std::deque<double> ts_history_;
    double drift_ns_per_sec_{0};

    /// Output queue for pull API.
    SensorQueue<SyncedFrame> output_q_;

    /// Callbacks.
    SyncedFrameCallback sync_cb_;
    DriftCallback       drift_cb_;

    /// Statistics.
    SyncStats stats_;
    std::atomic<uint64_t> frames_produced_{0};

    /// Thread.
    std::atomic<bool> running_{false};
    std::thread       thread_;
};

} // namespace thunderbird::data
