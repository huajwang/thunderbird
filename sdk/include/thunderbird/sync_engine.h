// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Time synchronization engine
// ─────────────────────────────────────────────────────────────────────────────
// Aligns data from three asynchronous sensor streams using a nearest-neighbour
// policy within a configurable tolerance window.
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/types.h"
#include "thunderbird/ring_buffer.h"

#include <algorithm>
#include <atomic>
#include <deque>
#include <mutex>
#include <thread>

namespace thunderbird {

struct SyncConfig {
    /// Maximum tolerable time difference between any two sensors (ns).
    int64_t tolerance_ns = 50'000'000;  // 50 ms default

    /// How often the sync thread checks for new bundles (ms).
    int     poll_interval_ms = 5;

    /// Reference sensor for time alignment.
    SensorType reference_sensor = SensorType::LiDAR;
};

/// The SyncEngine collects incoming sensor samples in short queues,
/// then tries to combine them into time-aligned SyncBundles.
class SyncEngine {
public:
    explicit SyncEngine(SyncConfig config = {})
        : config_(config) {}

    ~SyncEngine() { stop(); }

    // ── Feed functions (called from driver callbacks) ───────────────────────

    void feed_lidar(std::shared_ptr<const LidarFrame> f) {
        std::lock_guard<std::mutex> lk(mu_);
        lidar_q_.push_back(std::move(f));
        trim(lidar_q_);
    }

    void feed_imu(std::shared_ptr<const ImuSample> s) {
        std::lock_guard<std::mutex> lk(mu_);
        imu_q_.push_back(std::move(s));
        trim(imu_q_);
    }

    void feed_camera(std::shared_ptr<const CameraFrame> f) {
        std::lock_guard<std::mutex> lk(mu_);
        camera_q_.push_back(std::move(f));
        trim(camera_q_);
    }

    // ── Registration ────────────────────────────────────────────────────────

    void set_callback(SyncCallback cb) { callback_ = std::move(cb); }

    // ── Lifecycle ───────────────────────────────────────────────────────────

    void start() {
        if (running_.exchange(true)) return;
        thread_ = std::thread([this] { run(); });
    }

    void stop() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

    uint64_t bundles_produced() const { return bundles_produced_.load(); }

private:
    // Keep at most 64 samples in each queue (prevents unbounded growth).
    template <typename Deque>
    static void trim(Deque& dq, size_t max_size = 64) {
        while (dq.size() > max_size) dq.pop_front();
    }

    /// Timestamp accessor helpers
    static Timestamp ts(const std::shared_ptr<const LidarFrame>& p)  { return p->timestamp; }
    static Timestamp ts(const std::shared_ptr<const ImuSample>& p)   { return p->timestamp; }
    static Timestamp ts(const std::shared_ptr<const CameraFrame>& p) { return p->timestamp; }

    /// Find closest element to `ref` and return it + its index, or nullptr.
    template <typename Deque>
    static auto find_nearest(const Deque& dq, Timestamp ref, int64_t tol_ns)
        -> typename Deque::value_type
    {
        if (dq.empty()) return nullptr;
        typename Deque::value_type best = nullptr;
        int64_t best_diff = std::numeric_limits<int64_t>::max();

        for (auto& item : dq) {
            int64_t diff = std::abs(ts(item).nanoseconds - ref.nanoseconds);
            if (diff < best_diff) {
                best_diff = diff;
                best = item;
            }
        }
        if (best_diff > tol_ns) return nullptr;
        return best;
    }

    /// Remove items older than `cutoff` from the front of the deque.
    template <typename Deque>
    static void evict_before(Deque& dq, Timestamp cutoff) {
        while (!dq.empty() && ts(dq.front()) < cutoff) dq.pop_front();
    }

    void run() {
        while (running_) {
            try_match();
            std::this_thread::sleep_for(std::chrono::milliseconds(config_.poll_interval_ms));
        }
    }

    void try_match() {
        std::lock_guard<std::mutex> lk(mu_);
        if (lidar_q_.empty() || camera_q_.empty()) return;
        // IMU runs much faster — we always have samples once streaming starts.

        // Use the oldest un-consumed LiDAR frame as reference.
        auto ref = lidar_q_.front();
        Timestamp ref_ts = ts(ref);

        auto best_imu    = find_nearest(imu_q_,    ref_ts, config_.tolerance_ns);
        auto best_camera = find_nearest(camera_q_, ref_ts, config_.tolerance_ns);

        if (!best_camera) return; // camera hasn't arrived yet in tolerance

        // Build the bundle
        auto bundle = std::make_shared<SyncBundle>();
        bundle->reference_time = ref_ts;
        bundle->lidar  = ref;
        bundle->imu    = best_imu;   // may be nullptr if IMU not yet ready
        bundle->camera = best_camera;

        // Evict consumed data (everything at or before ref)
        Timestamp cutoff = {ref_ts.nanoseconds + 1};
        lidar_q_.pop_front();
        evict_before(imu_q_, cutoff);
        evict_before(camera_q_, cutoff);

        bundles_produced_.fetch_add(1);
        if (callback_) callback_(bundle);
    }

    SyncConfig  config_;
    SyncCallback callback_;

    std::mutex mu_;
    std::deque<std::shared_ptr<const LidarFrame>>  lidar_q_;
    std::deque<std::shared_ptr<const ImuSample>>   imu_q_;
    std::deque<std::shared_ptr<const CameraFrame>> camera_q_;

    std::atomic<bool>     running_{false};
    std::atomic<uint64_t> bundles_produced_{0};
    std::thread           thread_;
};

} // namespace thunderbird
