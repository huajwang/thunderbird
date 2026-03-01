// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — PerceptionEngine implementation
// ─────────────────────────────────────────────────────────────────────────────
//
// PImpl implementation owning three worker threads and four SPSC ring buffers.
// See perception_engine.h for the full thread model diagram.
//
// Key design constraints:
//   • feedSlamOutput() is lightweight: the SPSC push is lock-free, followed
//     by a condition_variable notify to wake T1.  The notify may involve a
//     syscall but keeps SLAM-thread blocking to a minimum.
//   • All heavy work (preprocessing, detection, tracking) happens on
//     dedicated threads — never on the caller's thread.
//   • GPU inference (T2) runs on its own thread with an exclusive CUDA
//     stream, so CUDA synchronisation never contends with other threads.
//
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/perception/perception_engine.h"
#include "thunderbird/perception/preprocessor.h"
#include "thunderbird/perception/object_detector.h"
#include "thunderbird/perception/multi_object_tracker.h"
#include "thunderbird/ring_buffer.h"

#include <atomic>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <thread>

namespace thunderbird::perception {

// ═════════════════════════════════════════════════════════════════════════════
//  Exponential moving average helper
// ═════════════════════════════════════════════════════════════════════════════

namespace {

class EMA {
public:
    explicit EMA(double alpha = 0.1) : alpha_(alpha) {}

    /// Called from exactly one writer thread.
    void update(double value) noexcept {
        if (!initialised_.load(std::memory_order_relaxed)) {
            value_.store(value, std::memory_order_relaxed);
            initialised_.store(true, std::memory_order_relaxed);
        } else {
            const double prev = value_.load(std::memory_order_relaxed);
            value_.store(alpha_ * value + (1.0 - alpha_) * prev,
                         std::memory_order_relaxed);
        }
    }

    /// Safe to call from any thread (relaxed atomic load).
    [[nodiscard]] double get() const noexcept {
        return value_.load(std::memory_order_relaxed);
    }

private:
    double              alpha_;
    std::atomic<double> value_{0.0};
    std::atomic<bool>   initialised_{false};
};

/// Monotonic wall-clock timer (microsecond precision).
inline double elapsed_ms(
    std::chrono::steady_clock::time_point start) noexcept
{
    const auto end = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(end - start).count();
}

} // anonymous namespace

// ═════════════════════════════════════════════════════════════════════════════
//  PImpl
// ═════════════════════════════════════════════════════════════════════════════

struct PerceptionEngine::Impl {
    // ── Configuration (immutable after initialize) ──────────────────────
    PerceptionConfig config;

    // ── Pipeline stages ─────────────────────────────────────────────────
    std::unique_ptr<PointCloudPreprocessor> preprocessor;
    std::unique_ptr<ObjectDetector>         detector;
    std::unique_ptr<MultiObjectTracker>     tracker;

    // ── Lock-free ring buffers (SPSC) ───────────────────────────────────
    // perception_ring_: SLAM → T1 (preprocessor)
    RingBuffer<std::shared_ptr<const odom::SlamOutput>,
               PipelineConfig::kPerceptionRingCapacity>  perception_ring;

    // detection_ring_: T1 → T2 (detector)
    RingBuffer<std::shared_ptr<DetectionInput>,
               PipelineConfig::kDetectionRingCapacity>   detection_ring;

    // tracking_ring_: T2 → T3 (tracker)
    RingBuffer<std::shared_ptr<const DetectionFrame>,
               PipelineConfig::kTrackingRingCapacity>    tracking_ring;

    // output_ring_: T3 → user thread
    RingBuffer<std::shared_ptr<const TrackedObjectList>,
               PipelineConfig::kOutputRingCapacity>      output_ring;

    // Optional: intermediate detection output for diagnostics.
    RingBuffer<std::shared_ptr<const DetectionFrame>,
               PipelineConfig::kOutputRingCapacity>      detection_output_ring;

    // ── Worker threads ──────────────────────────────────────────────────
    std::thread t1_preprocessor;
    std::thread t2_detector;
    std::thread t3_tracker;

    std::atomic<bool> initialized{false};
    std::atomic<bool> running{false};
    std::atomic<bool> shutdown_requested{false};

    // ── Wake conditions ─────────────────────────────────────────────────
    std::mutex               t1_wake_mu;
    std::condition_variable  t1_wake_cv;
    std::mutex               t2_wake_mu;
    std::condition_variable  t2_wake_cv;
    std::mutex               t3_wake_mu;
    std::condition_variable  t3_wake_cv;

    // ── Rate control ────────────────────────────────────────────────────
    int64_t min_frame_interval_ns{0};  // 0 = no throttle
    int64_t last_processed_ts{0};

    // ── Callbacks (registered before start, read from worker threads) ───
    std::mutex                cb_mu;
    TrackedObjectCallback     on_tracked_cb;
    DetectionCallback         on_detection_cb;

    // ── Statistics (atomics for cross-thread reads) ─────────────────────
    std::atomic<uint64_t> stat_frames_received{0};
    std::atomic<uint64_t> stat_frames_processed{0};
    std::atomic<uint64_t> stat_frames_dropped{0};
    std::atomic<uint32_t> stat_active_tracks{0};

    EMA ema_preprocess{0.1};
    EMA ema_detection{0.1};
    EMA ema_tracking{0.1};
    EMA ema_total{0.1};

    // Track count cache (atomic for cross-thread stats reads).

    // ── Constructor ─────────────────────────────────────────────────────
    Impl() = default;

    // ═════════════════════════════════════════════════════════════════════
    //  T1 — Preprocessor thread
    // ═════════════════════════════════════════════════════════════════════

    void preprocessorLoop() {
        while (!shutdown_requested.load(std::memory_order_acquire)) {
            auto slam_out = perception_ring.pop();
            if (!slam_out) {
                // Nothing to process — sleep briefly.
                std::unique_lock<std::mutex> lk(t1_wake_mu);
                t1_wake_cv.wait_for(lk, std::chrono::microseconds(500));
                continue;
            }

            const auto& output = *slam_out;
            if (!output || !output->deskewed_cloud) continue;

            // Rate throttle: skip frames if exceeding max_inference_rate_hz.
            if (min_frame_interval_ns > 0) {
                const int64_t ts = output->timestamp_ns;
                if (ts - last_processed_ts < min_frame_interval_ns) {
                    continue;  // skip this frame
                }
                last_processed_ts = ts;
            }

            const auto t_start = std::chrono::steady_clock::now();

            auto det_input = std::make_shared<DetectionInput>(
                preprocessor->process(
                    output->deskewed_cloud,
                    output->pose,
                    output->timestamp_ns));

            const double preproc_ms = elapsed_ms(t_start);
            ema_preprocess.update(preproc_ms);

            detection_ring.push(std::move(det_input));
            t2_wake_cv.notify_one();
        }
    }

    // ═════════════════════════════════════════════════════════════════════
    //  T2 — Detector thread (GPU or CPU)
    // ═════════════════════════════════════════════════════════════════════

    void detectorLoop() {
        while (!shutdown_requested.load(std::memory_order_acquire)) {
            auto input = detection_ring.pop();
            if (!input) {
                std::unique_lock<std::mutex> lk(t2_wake_mu);
                t2_wake_cv.wait_for(lk, std::chrono::microseconds(500));
                continue;
            }

            const auto t_start = std::chrono::steady_clock::now();

            DetectionFrame frame = detector->detect(**input);

            const double detect_ms = elapsed_ms(t_start);
            ema_detection.update(detect_ms);

            auto frame_ptr = std::make_shared<const DetectionFrame>(std::move(frame));

            // Publish intermediate detections if configured.
            if (config.pipeline.publish_intermediate_detections) {
                detection_output_ring.push(frame_ptr);

                // Fire detection callback.  Copy under the lock,
                // then invoke outside to avoid stalling T2 if the
                // user callback is slow or re-registers callbacks.
                DetectionCallback det_cb;
                {
                    std::lock_guard<std::mutex> lk(cb_mu);
                    det_cb = on_detection_cb;
                }
                if (det_cb) {
                    det_cb(frame_ptr);
                }
            }

            tracking_ring.push(std::move(frame_ptr));
            t3_wake_cv.notify_one();
        }
    }

    // ═════════════════════════════════════════════════════════════════════
    //  T3 — Tracker thread (Kalman + Hungarian)
    // ═════════════════════════════════════════════════════════════════════

    void trackerLoop() {
        while (!shutdown_requested.load(std::memory_order_acquire)) {
            auto det_frame = tracking_ring.pop();
            if (!det_frame) {
                std::unique_lock<std::mutex> lk(t3_wake_mu);
                t3_wake_cv.wait_for(lk, std::chrono::microseconds(500));
                continue;
            }

            const auto t_start = std::chrono::steady_clock::now();

            auto tracked = tracker->update(**det_frame);

            const double track_ms = elapsed_ms(t_start);
            ema_tracking.update(track_ms);
            ema_total.update(ema_preprocess.get() + ema_detection.get() + track_ms);

            // Populate pipeline-level diagnostics.
            tracked.preprocess_ms  = ema_preprocess.get();
            tracked.detection_ms   = ema_detection.get();
            tracked.tracking_ms    = track_ms;
            tracked.input_points   = (*det_frame)->input_points;
            tracked.filtered_points= (*det_frame)->filtered_points;
            tracked.num_clusters   = (*det_frame)->cluster_count;
            tracked.num_detections = static_cast<uint32_t>((*det_frame)->detections.size());

            stat_frames_processed.fetch_add(1, std::memory_order_relaxed);
            stat_active_tracks.store(
                tracker->activeTrackCount(),
                std::memory_order_relaxed);

            auto list_ptr = std::make_shared<const TrackedObjectList>(std::move(tracked));

            output_ring.push(list_ptr);

            // Fire tracked-object callback.  Copy under the lock,
            // then invoke outside to avoid stalling T3 if the
            // user callback is slow or re-registers callbacks.
            TrackedObjectCallback trk_cb;
            {
                std::lock_guard<std::mutex> lk(cb_mu);
                trk_cb = on_tracked_cb;
            }
            if (trk_cb) {
                trk_cb(list_ptr);
            }
        }
    }
};

// ═════════════════════════════════════════════════════════════════════════════
//  Public API forwarding
// ═════════════════════════════════════════════════════════════════════════════

PerceptionEngine::PerceptionEngine()
    : impl_(std::make_unique<Impl>()) {}

PerceptionEngine::~PerceptionEngine() {
    shutdown();
}

bool PerceptionEngine::initialize(const PerceptionConfig& config) {
    impl_->config = config;

    // ── Create preprocessor ─────────────────────────────────────────────
    impl_->preprocessor = std::make_unique<PointCloudPreprocessor>(
        config.preprocessor);

    // ── Create detector (factory selects backend) ───────────────────────
    impl_->detector = ObjectDetector::create(config);
    if (!impl_->detector) return false;

    if (!impl_->detector->initialize(config)) {
        return false;
    }

    // ── Create tracker ──────────────────────────────────────────────────
    impl_->tracker = std::make_unique<MultiObjectTracker>(config.tracker);

    // ── Rate control ────────────────────────────────────────────────────
    if (config.pipeline.max_inference_rate_hz > 0) {
        impl_->min_frame_interval_ns = static_cast<int64_t>(
            1.0e9 / config.pipeline.max_inference_rate_hz);
    }

    impl_->initialized.store(true, std::memory_order_release);
    return true;
}

void PerceptionEngine::start() {
    if (impl_->running.load(std::memory_order_relaxed)) return;

    assert(impl_->initialized.load(std::memory_order_acquire) &&
           "PerceptionEngine::start() called before successful initialize()");

    impl_->shutdown_requested.store(false, std::memory_order_release);
    impl_->running.store(true, std::memory_order_release);

    impl_->t1_preprocessor = std::thread([this] { impl_->preprocessorLoop(); });
    impl_->t2_detector     = std::thread([this] { impl_->detectorLoop(); });
    impl_->t3_tracker      = std::thread([this] { impl_->trackerLoop(); });
}

void PerceptionEngine::stop() {
    if (!impl_->running.load(std::memory_order_relaxed)) return;

    // Mark not-running FIRST so that feedSlamOutput() stops pushing
    // into perception_ring before we join threads and drain buffers.
    // This preserves the SPSC contract (single producer quiesced
    // before consumer-side clear).
    impl_->running.store(false, std::memory_order_release);

    impl_->shutdown_requested.store(true, std::memory_order_release);

    // Wake all threads so they can observe the shutdown flag.
    impl_->t1_wake_cv.notify_all();
    impl_->t2_wake_cv.notify_all();
    impl_->t3_wake_cv.notify_all();

    if (impl_->t1_preprocessor.joinable()) impl_->t1_preprocessor.join();
    if (impl_->t2_detector.joinable())     impl_->t2_detector.join();
    if (impl_->t3_tracker.joinable())      impl_->t3_tracker.join();

    // Drain all ring buffers so a subsequent start() doesn't process
    // stale frames.  drain_and_reset() also releases the underlying
    // shared_ptr slots (unlike a plain head/tail reset) so large
    // point-cloud frames are freed immediately.
    impl_->perception_ring.drain_and_reset();
    impl_->detection_ring.drain_and_reset();
    impl_->tracking_ring.drain_and_reset();
    impl_->output_ring.drain_and_reset();
    impl_->detection_output_ring.drain_and_reset();

    // Reset per-run rate limiting state so a subsequent start() begins cleanly.
    impl_->last_processed_ts = 0;
}

void PerceptionEngine::shutdown() {
    stop();

    if (impl_->detector) {
        impl_->detector->teardown();
        impl_->detector.reset();
    }
    impl_->tracker.reset();
    impl_->preprocessor.reset();

    // Mark uninitialised so a subsequent start() cannot pass the
    // initialized assert and dereference null stage pointers.
    impl_->initialized.store(false, std::memory_order_release);
}

bool PerceptionEngine::isRunning() const noexcept {
    return impl_->running.load(std::memory_order_acquire);
}

// ── SLAM ingress ────────────────────────────────────────────────────────────

void PerceptionEngine::feedSlamOutput(
    std::shared_ptr<const odom::SlamOutput> output)
{
    // Ignore frames when the pipeline is not running.  Without this
    // guard, frames pushed after stop() (or before start()) would
    // accumulate in the ring and be processed as stale data on the
    // next start().
    if (!impl_->running.load(std::memory_order_acquire))
        return;

    impl_->stat_frames_received.fetch_add(1, std::memory_order_relaxed);

    // Lock-free push.  If ring is full, oldest is dropped.
    impl_->perception_ring.push(std::move(output));
    impl_->t1_wake_cv.notify_one();
}

// ── Pull API ────────────────────────────────────────────────────────────────

bool PerceptionEngine::getDetectedObjects(
    std::shared_ptr<const TrackedObjectList>& out)
{
    auto result = impl_->output_ring.pop();
    if (!result) return false;
    out = std::move(*result);
    return true;
}

bool PerceptionEngine::getLatestDetections(
    std::shared_ptr<const DetectionFrame>& out)
{
    auto result = impl_->detection_output_ring.pop();
    if (!result) return false;
    out = std::move(*result);
    return true;
}

// ── Callback API ────────────────────────────────────────────────────────────

void PerceptionEngine::onTrackedObjects(TrackedObjectCallback cb) {
    std::lock_guard<std::mutex> lk(impl_->cb_mu);
    impl_->on_tracked_cb = std::move(cb);
}

void PerceptionEngine::onDetections(DetectionCallback cb) {
    std::lock_guard<std::mutex> lk(impl_->cb_mu);
    impl_->on_detection_cb = std::move(cb);
}

// ── Diagnostics ─────────────────────────────────────────────────────────────

PerceptionEngine::Stats PerceptionEngine::stats() const noexcept {
    Stats s;
    s.frames_received  = impl_->stat_frames_received.load(std::memory_order_relaxed);
    s.frames_processed = impl_->stat_frames_processed.load(std::memory_order_relaxed);
    s.frames_dropped   = impl_->perception_ring.dropped();
    // EMA reads use relaxed atomics — values may be slightly stale
    // but free of undefined behaviour.  Acceptable for diagnostics.
    s.avg_preprocess_ms = impl_->ema_preprocess.get();
    s.avg_detection_ms  = impl_->ema_detection.get();
    s.avg_tracking_ms   = impl_->ema_tracking.get();
    s.avg_total_ms      = impl_->ema_total.get();
    s.active_tracks    = impl_->stat_active_tracks.load(std::memory_order_relaxed);
    s.detector_backend = impl_->detector ? impl_->detector->name() : "none";
    return s;
}

size_t PerceptionEngine::dropCount() const noexcept {
    return impl_->perception_ring.dropped();
}

} // namespace thunderbird::perception
