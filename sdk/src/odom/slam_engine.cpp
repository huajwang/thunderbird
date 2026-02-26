// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — AcmeSlamEngine implementation
// ─────────────────────────────────────────────────────────────────────────────
//
// This file contains the full PImpl implementation of AcmeSlamEngine.
// All Eigen / ikd-Tree / ESIKF dependencies are confined to this translation
// unit and never appear in the public header.
//
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/odom/slam_engine.h"
#include "thunderbird/odom/slam_time_sync.h"
#include "thunderbird/ring_buffer.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <thread>

// ─────────────────────────────────────────────────────────────────────────────
// In a full build these would be the real FAST-LIVO2 headers:
//
//   #include <Eigen/Core>
//   #include <Eigen/Geometry>
//   #include "ikd-Tree/ikd_Tree.h"
//   #include "fast_livo2/esikf.h"
//   #include "fast_livo2/preintegration.h"
//
// For now we provide forward declarations and placeholder types so that the
// engine skeleton compiles and the public API can be exercised.
// ─────────────────────────────────────────────────────────────────────────────

namespace thunderbird::odom {

// ═════════════════════════════════════════════════════════════════════════════
//  Placeholder types for ESIKF / ikd-Tree (replaced by real impls)
// ═════════════════════════════════════════════════════════════════════════════

namespace internal {

/// Placeholder ESIKF state — will hold Eigen matrices in the real build.
struct EsikfState {
    // Rotation (as quaternion w,x,y,z)
    double quat[4]{1, 0, 0, 0};
    double pos[3]{};
    double vel[3]{};
    double bg[3]{};   // gyro bias
    double ba[3]{};   // accel bias
    double grav[3]{0, 0, -9.81};

    // 24×24 covariance (row-major, placeholder flat array)
    double cov[kStateDim * kStateDim]{};

    // Marginal 6×6 for external consumption
    double cov6x6[36]{};
};

/// Placeholder ikd-Tree wrapper.
struct IkdTreeWrapper {
    size_t num_points{0};
    double center[3]{};
    double radius{0};

    void clear() { num_points = 0; radius = 0; }
    void insert(const PointCloudFrame& /*cloud*/) {
        // TODO: real ikd-Tree insertion
        num_points += 100; // stub
    }
    void rebalance() { /* TODO */ }
};

/// Placeholder ESIKF propagation & update.
struct EsikfEngine {
    EsikfState state;
    int last_iterations{0};
    double last_residual{0};
    uint32_t last_correspondences{0};

    void set_noise_model(const ImuNoiseModel& /*noise*/) { /* TODO */ }
    void set_extrinsic(const ExtrinsicCalibration& /*ext*/) { /* TODO */ }
    void set_params(const EsikfConfig& /*cfg*/) { /* TODO */ }

    void propagate(const ImuSample& /*sample*/, double /*dt*/) {
        // TODO: real IMU propagation
        // state += F * state + G * u; P = F*P*F' + Q
    }

    bool update(const PointCloudFrame& /*deskewed*/,
                const IkdTreeWrapper& /*map*/,
                const EsikfConfig& cfg) {
        // TODO: real ESIKF iterated update
        // Returns true if update was accepted
        last_iterations = cfg.max_iterations;
        last_residual = 0.001;
        last_correspondences = 500;
        return true;
    }

    void reset() {
        state = EsikfState{};
        last_iterations = 0;
        last_residual = 0;
        last_correspondences = 0;
    }
};

/// Placeholder point deskewer.
struct PointDeskewer {
    /// Deskew a raw cloud using the IMU trajectory.
    static std::shared_ptr<PointCloudFrame> deskew(
        const PointCloudFrame& raw,
        const std::vector<ImuSample>& /*imu_block*/,
        const EsikfState& /*state*/)
    {
        // TODO: real IMU-based motion compensation
        // For now, return a copy marked as deskewed.
        auto out = std::make_shared<PointCloudFrame>();
        out->timestamp_ns = raw.timestamp_ns;
        out->sequence     = raw.sequence;
        out->is_deskewed  = true;
        out->points       = raw.points;  // real impl transforms each point
        return out;
    }
};

/// Gravity alignment from static IMU data.
struct GravityAligner {
    double accel_sum[3]{};
    int    count{0};

    void addSample(const ImuSample& s) {
        for (int i = 0; i < 3; ++i) accel_sum[i] += s.accel[i];
        ++count;
    }

    bool ready(const InitConfig& cfg) const {
        return count >= cfg.init_min_imu_samples;
    }

    /// Estimate gravity direction and initial orientation.
    /// Returns false if the IMU data was too dynamic (not static).
    bool align(EsikfState& state, const InitConfig& /*cfg*/) {
        if (count == 0) return false;
        const double n = static_cast<double>(count);
        double mean_a[3];
        for (int i = 0; i < 3; ++i) mean_a[i] = accel_sum[i] / n;

        // Gravity magnitude
        const double g_mag = std::sqrt(
            mean_a[0]*mean_a[0] + mean_a[1]*mean_a[1] + mean_a[2]*mean_a[2]);
        if (g_mag < 1.0) return false; // sensor problem

        // Normalised gravity in body frame → derive initial orientation.
        // Simplified: assume z-down gravity world frame.
        state.grav[0] = 0;
        state.grav[1] = 0;
        state.grav[2] = -g_mag;

        // TODO: real rotation alignment from gravity vector
        state.quat[0] = 1; state.quat[1] = 0;
        state.quat[2] = 0; state.quat[3] = 0;
        return true;
    }

    void reset() {
        accel_sum[0] = accel_sum[1] = accel_sum[2] = 0;
        count = 0;
    }
};

} // namespace internal

// ═════════════════════════════════════════════════════════════════════════════
//  Timestamped IMU wrapper for lock-free ring buffer
// ═════════════════════════════════════════════════════════════════════════════

struct ImuIngress {
    ImuSample sample;
    int64_t   host_ns{0};
};

struct CloudIngress {
    std::shared_ptr<const PointCloudFrame> cloud;
    int64_t host_ns{0};
};

// ═════════════════════════════════════════════════════════════════════════════
//  PImpl
// ═════════════════════════════════════════════════════════════════════════════

struct AcmeSlamEngine::Impl {
    // ── Configuration (immutable after initialize) ──────────────────────
    SlamEngineConfig config;

    // ── Lock-free ingress queues (SPSC) ─────────────────────────────────
    RingBuffer<ImuIngress,   SlamEngineConfig::kImuRingCapacity>    imu_ring;
    RingBuffer<CloudIngress, SlamEngineConfig::kCloudRingCapacity>  cloud_ring;

    // ── Lock-free egress queues (SPSC) ──────────────────────────────────
    RingBuffer<std::shared_ptr<const SlamOutput>, SlamEngineConfig::kOutputRingCapacity> output_ring;
    RingBuffer<std::shared_ptr<const Pose6D>,     SlamEngineConfig::kPoseRingCapacity>   pose_ring;

    // ── Time synchronisation ────────────────────────────────────────────
    std::unique_ptr<SlamTimeSync> time_sync;

    // ── ESIKF internals (only accessed from worker thread) ──────────────
    internal::EsikfEngine   esikf;
    internal::IkdTreeWrapper ikd_tree;
    internal::GravityAligner gravity_aligner;
    int64_t                  last_imu_ts{0};
    uint32_t                 scan_counter{0};

    // ── Worker thread ───────────────────────────────────────────────────
    std::thread              worker;
    std::atomic<bool>        running{false};
    std::atomic<bool>        shutdown_requested{false};
    std::atomic<bool>        reset_requested{false};

    // Wake condition: worker sleeps when no data is available.
    std::mutex               wake_mu;
    std::condition_variable  wake_cv;

    // ── Tracking status (atomic for cross-thread visibility) ────────────
    std::atomic<TrackingStatus> tracking_status{TrackingStatus::Initializing};

    // ── Rolling statistics ──────────────────────────────────────────────
    struct Stats {
        std::atomic<uint64_t> imu_processed{0};
        std::atomic<uint64_t> scans_processed{0};
        std::atomic<double>   avg_propagation_us{0};
        std::atomic<double>   avg_update_ms{0};
    } stats;

    // ── User callbacks (set before start, read from worker) ─────────────
    // Protected by cb_mu only during registration; worker reads after fence.
    std::mutex               cb_mu;
    SlamOutputCallback       on_output_cb;
    PoseCallback             on_pose_cb;
    std::function<void(TrackingStatus)> on_status_cb;

    // ── Constructor ─────────────────────────────────────────────────────
    Impl() = default;

    // ── Worker loop ─────────────────────────────────────────────────────

    void workerLoop() {
        while (!shutdown_requested.load(std::memory_order_acquire)) {

            // ── Handle reset request ────────────────────────────────
            if (reset_requested.load(std::memory_order_acquire)) {
                performReset();
                reset_requested.store(false, std::memory_order_release);
            }

            bool did_work = false;

            // ── Phase 1: Drain IMU ring → time sync ─────────────────
            did_work |= drainImuRing();

            // ── Phase 2: Drain cloud ring → time sync ───────────────
            did_work |= drainCloudRing();

            // ── Phase 3: Try to assemble & process a scan ───────────
            did_work |= tryProcessScan();

            // ── Sleep if idle ───────────────────────────────────────
            if (!did_work) {
                std::unique_lock<std::mutex> lk(wake_mu);
                wake_cv.wait_for(lk, std::chrono::microseconds(500));
            }
        }
    }

    // ── Phase 1: drain IMU ingress ──────────────────────────────────────

    bool drainImuRing() {
        bool did_work = false;
        while (auto item = imu_ring.pop()) {
            did_work = true;
            const auto& [sample, host_ns] = *item;
            time_sync->feed_imu(sample, host_ns);

            // If still initialising, accumulate for gravity alignment.
            if (tracking_status.load(std::memory_order_relaxed) ==
                TrackingStatus::Initializing) {
                gravity_aligner.addSample(sample);
                if (gravity_aligner.ready(config.init)) {
                    if (gravity_aligner.align(esikf.state, config.init)) {
                        transitionStatus(TrackingStatus::Converging);
                    }
                }
                last_imu_ts = sample.timestamp_ns;
                continue;
            }

            // Propagate ESIKF for each IMU sample.
            if (last_imu_ts > 0) {
                const double dt_s =
                    static_cast<double>(sample.timestamp_ns - last_imu_ts)
                    / 1.0e9;
                if (dt_s > 0 && dt_s < 1.0) {
                    using clock = std::chrono::steady_clock;
                    const auto t0 = clock::now();
                    esikf.propagate(sample, dt_s);
                    const auto t1 = clock::now();
                    const double us =
                        std::chrono::duration<double, std::micro>(t1 - t0)
                            .count();

                    // Rolling average.
                    const double n = static_cast<double>(
                        stats.imu_processed.load(std::memory_order_relaxed) + 1);
                    const double prev =
                        stats.avg_propagation_us.load(std::memory_order_relaxed);
                    stats.avg_propagation_us.store(
                        prev + (us - prev) / n, std::memory_order_relaxed);
                }
            }
            last_imu_ts = sample.timestamp_ns;
            stats.imu_processed.fetch_add(1, std::memory_order_relaxed);

            // ── Publish propagated pose ─────────────────────────────
            if (config.publish_propagated_poses) {
                auto pose = stateTopose(sample.timestamp_ns, false);
                pose_ring.push(pose);

                std::lock_guard<std::mutex> lk(cb_mu);
                if (on_pose_cb) on_pose_cb(pose);
            }
        }
        return did_work;
    }

    // ── Phase 2: drain cloud ingress ────────────────────────────────────

    bool drainCloudRing() {
        bool did_work = false;
        while (auto item = cloud_ring.pop()) {
            did_work = true;
            const auto& [cloud, host_ns] = *item;
            time_sync->feed_lidar(cloud, host_ns);
        }
        return did_work;
    }

    // ── Phase 3: assemble ScanMeasurement → ESIKF update ───────────────

    bool tryProcessScan() {
        auto meas_opt = time_sync->poll_next_measurement();
        if (!meas_opt) return false;

        auto& meas = *meas_opt;

        // Skip processing if still initializing.
        if (tracking_status.load(std::memory_order_relaxed) ==
            TrackingStatus::Initializing) {
            return true;
        }

        using clock = std::chrono::steady_clock;
        const auto t0 = clock::now();

        // ── 3a: Deskew ──────────────────────────────────────────────
        std::shared_ptr<PointCloudFrame> deskewed;
        if (config.deskew.enable && meas.scan) {
            deskewed = internal::PointDeskewer::deskew(
                *meas.scan, meas.imu_block, esikf.state);
        } else if (meas.scan) {
            // No deskewing — use raw cloud.
            deskewed = std::make_shared<PointCloudFrame>(*meas.scan);
        }

        if (!deskewed || deskewed->points.empty()) return true;

        // ── 3b: ESIKF iterated update ───────────────────────────────
        bool update_ok = esikf.update(*deskewed, ikd_tree, config.esikf);

        // ── 3c: Insert into ikd-Tree ────────────────────────────────
        if (update_ok) {
            ikd_tree.insert(*deskewed);

            if (scan_counter > 0 &&
                (scan_counter % static_cast<uint32_t>(
                    config.map.tree_rebalance_interval)) == 0) {
                ikd_tree.rebalance();
            }
        }

        const auto t1 = clock::now();
        const double update_ms =
            std::chrono::duration<double, std::milli>(t1 - t0).count();

        // ── 3d: Update tracking status ──────────────────────────────
        updateTrackingStatus(update_ok);

        // ── 3e: Build and publish SlamOutput ────────────────────────
        auto output = std::make_shared<SlamOutput>();
        output->timestamp_ns    = meas.scan_end_ns;
        output->pose            = *stateTopose(meas.scan_end_ns, true);
        output->deskewed_cloud  = std::move(deskewed);
        output->map_info.total_points = ikd_tree.num_points;
        output->map_info.center = {
            ikd_tree.center[0], ikd_tree.center[1], ikd_tree.center[2]};
        output->map_info.radius = ikd_tree.radius;
        output->esikf_iterations   = esikf.last_iterations;
        output->esikf_residual     = esikf.last_residual;
        output->correspondences    = esikf.last_correspondences;
        output->update_latency_ms  = update_ms;

        output_ring.push(output);

        // Also publish the corrected Pose6D
        auto corrected_pose = stateTopose(meas.scan_end_ns, true);
        pose_ring.push(corrected_pose);

        // Fire callbacks.
        {
            std::lock_guard<std::mutex> lk(cb_mu);
            if (on_output_cb) on_output_cb(output);
            if (on_pose_cb) on_pose_cb(corrected_pose);
        }

        // ── 3f: Update stats ────────────────────────────────────────
        ++scan_counter;
        stats.scans_processed.fetch_add(1, std::memory_order_relaxed);
        {
            const double n = static_cast<double>(
                stats.scans_processed.load(std::memory_order_relaxed));
            const double prev =
                stats.avg_update_ms.load(std::memory_order_relaxed);
            stats.avg_update_ms.store(
                prev + (update_ms - prev) / n, std::memory_order_relaxed);
        }

        return true;
    }

    // ── Helpers ─────────────────────────────────────────────────────────

    /// Convert current ESIKF state to a public Pose6D.
    std::shared_ptr<const Pose6D> stateTopose(int64_t ts, bool corrected) {
        auto p = std::make_shared<Pose6D>();
        p->timestamp_ns = ts;
        for (int i = 0; i < 4; ++i) p->quaternion[i]    = esikf.state.quat[i];
        for (int i = 0; i < 3; ++i) p->position[i]      = esikf.state.pos[i];
        for (int i = 0; i < 3; ++i) p->velocity[i]      = esikf.state.vel[i];
        for (int i = 0; i < 3; ++i) p->gyro_bias[i]     = esikf.state.bg[i];
        for (int i = 0; i < 3; ++i) p->accel_bias[i]    = esikf.state.ba[i];
        for (int i = 0; i < 3; ++i) p->gravity[i]       = esikf.state.grav[i];
        for (int i = 0; i < 36;++i) p->covariance_6x6[i]= esikf.state.cov6x6[i];
        p->tracking_status = tracking_status.load(std::memory_order_relaxed);
        p->is_corrected = corrected;
        return p;
    }

    /// Transition tracking status and fire callback.
    void transitionStatus(TrackingStatus next) {
        const auto prev = tracking_status.exchange(
            next, std::memory_order_release);
        if (prev != next) {
            std::lock_guard<std::mutex> lk(cb_mu);
            if (on_status_cb) on_status_cb(next);
        }
    }

    /// Update tracking status based on ESIKF update outcome.
    void updateTrackingStatus(bool update_ok) {
        auto cur = tracking_status.load(std::memory_order_relaxed);

        if (!update_ok) {
            if (cur == TrackingStatus::Tracking ||
                cur == TrackingStatus::Converging) {
                transitionStatus(TrackingStatus::Degraded);
            } else if (cur == TrackingStatus::Degraded) {
                transitionStatus(TrackingStatus::Lost);
            }
            return;
        }

        // Successful update.
        if (cur == TrackingStatus::Converging && scan_counter >= 5) {
            transitionStatus(TrackingStatus::Tracking);
        } else if (cur == TrackingStatus::Degraded) {
            transitionStatus(TrackingStatus::Tracking);
        }
    }

    /// Perform a full internal reset (called from worker thread).
    void performReset() {
        // Drain all queues.
        while (imu_ring.pop()) {}
        while (cloud_ring.pop()) {}
        while (output_ring.pop()) {}
        while (pose_ring.pop()) {}

        // Reset time sync.
        time_sync->reset();

        // Reset estimator state.
        esikf.reset();
        ikd_tree.clear();
        gravity_aligner.reset();
        last_imu_ts = 0;
        scan_counter = 0;

        // Reset stats.
        stats.imu_processed.store(0, std::memory_order_relaxed);
        stats.scans_processed.store(0, std::memory_order_relaxed);
        stats.avg_propagation_us.store(0, std::memory_order_relaxed);
        stats.avg_update_ms.store(0, std::memory_order_relaxed);

        transitionStatus(TrackingStatus::Initializing);
    }
};

// ═════════════════════════════════════════════════════════════════════════════
//  Public API implementation
// ═════════════════════════════════════════════════════════════════════════════

AcmeSlamEngine::AcmeSlamEngine()
    : impl_(std::make_unique<Impl>()) {}

AcmeSlamEngine::~AcmeSlamEngine() {
    if (impl_ && impl_->running.load(std::memory_order_acquire)) {
        shutdown();
    }
}

bool AcmeSlamEngine::initialize(const SlamEngineConfig& config) {
    if (impl_->running.load(std::memory_order_acquire)) {
        return false;  // already running
    }

    // ── Store configuration ─────────────────────────────────────────────
    impl_->config = config;

    // ── Configure time sync ─────────────────────────────────────────────
    SlamTimeSyncConfig tsc;
    tsc.imu_rate_hz              = config.imu_rate_hz;
    tsc.lidar_rate_hz            = config.lidar_rate_hz;
    tsc.enable_drift_compensation = config.enable_drift_compensation;
    tsc.sort_window_ns           = config.sort_window_ns;
    impl_->time_sync = std::make_unique<SlamTimeSync>(tsc);

    // ── Configure ESIKF ─────────────────────────────────────────────────
    impl_->esikf.set_noise_model(config.imu_noise);
    impl_->esikf.set_extrinsic(config.extrinsic);
    impl_->esikf.set_params(config.esikf);

    // ── Reset state ─────────────────────────────────────────────────────
    impl_->esikf.reset();
    impl_->ikd_tree.clear();
    impl_->gravity_aligner.reset();
    impl_->last_imu_ts = 0;
    impl_->scan_counter = 0;
    impl_->tracking_status.store(
        TrackingStatus::Initializing, std::memory_order_release);

    // ── Clear queues ────────────────────────────────────────────────────
    impl_->imu_ring.clear();
    impl_->cloud_ring.clear();
    impl_->output_ring.clear();
    impl_->pose_ring.clear();

    // ── Spawn worker thread ─────────────────────────────────────────────
    impl_->shutdown_requested.store(false, std::memory_order_release);
    impl_->reset_requested.store(false, std::memory_order_release);
    impl_->running.store(true, std::memory_order_release);

    impl_->worker = std::thread([this]() {
        impl_->workerLoop();
    });

    return true;
}

void AcmeSlamEngine::reset() {
    if (!impl_->running.load(std::memory_order_acquire)) return;
    impl_->reset_requested.store(true, std::memory_order_release);
    impl_->wake_cv.notify_one();
}

void AcmeSlamEngine::shutdown() {
    if (!impl_->running.load(std::memory_order_acquire)) return;

    impl_->shutdown_requested.store(true, std::memory_order_release);
    impl_->wake_cv.notify_one();

    if (impl_->worker.joinable()) {
        impl_->worker.join();
    }

    impl_->running.store(false, std::memory_order_release);
}

bool AcmeSlamEngine::isRunning() const noexcept {
    return impl_->running.load(std::memory_order_acquire);
}

// ── Data ingress ────────────────────────────────────────────────────────────

void AcmeSlamEngine::feedImu(const ImuSample& sample, int64_t host_ns) {
    impl_->imu_ring.push(ImuIngress{sample, host_ns});
    impl_->wake_cv.notify_one();
}

void AcmeSlamEngine::feedPointCloud(
    std::shared_ptr<const PointCloudFrame> cloud,
    int64_t host_ns)
{
    impl_->cloud_ring.push(CloudIngress{std::move(cloud), host_ns});
    impl_->wake_cv.notify_one();
}

// ── Data egress ─────────────────────────────────────────────────────────────

bool AcmeSlamEngine::getLatestOutput(SlamOutput& out) {
    std::shared_ptr<const SlamOutput> latest;
    // Drain to get the most recent.
    while (auto item = impl_->output_ring.pop()) {
        latest = std::move(*item);
    }
    if (!latest) return false;
    out = *latest;
    return true;
}

size_t AcmeSlamEngine::drainOutputs(std::vector<SlamOutput>& outputs) {
    outputs.clear();
    while (auto item = impl_->output_ring.pop()) {
        outputs.push_back(**item);
    }
    return outputs.size();
}

bool AcmeSlamEngine::getLatestPose(Pose6D& pose) {
    std::shared_ptr<const Pose6D> latest;
    while (auto item = impl_->pose_ring.pop()) {
        latest = std::move(*item);
    }
    if (!latest) return false;
    pose = *latest;
    return true;
}

// ── Callbacks ───────────────────────────────────────────────────────────────

void AcmeSlamEngine::onSlamOutput(SlamOutputCallback cb) {
    std::lock_guard<std::mutex> lk(impl_->cb_mu);
    impl_->on_output_cb = std::move(cb);
}

void AcmeSlamEngine::onPose(PoseCallback cb) {
    std::lock_guard<std::mutex> lk(impl_->cb_mu);
    impl_->on_pose_cb = std::move(cb);
}

void AcmeSlamEngine::onStatusChange(std::function<void(TrackingStatus)> cb) {
    std::lock_guard<std::mutex> lk(impl_->cb_mu);
    impl_->on_status_cb = std::move(cb);
}

// ── Diagnostics ─────────────────────────────────────────────────────────────

TrackingStatus AcmeSlamEngine::status() const noexcept {
    return impl_->tracking_status.load(std::memory_order_acquire);
}

OdometryStats AcmeSlamEngine::stats() const noexcept {
    OdometryStats s;
    s.imu_samples_processed  = impl_->stats.imu_processed.load(
        std::memory_order_relaxed);
    s.lidar_scans_processed  = impl_->stats.scans_processed.load(
        std::memory_order_relaxed);
    s.avg_imu_propagation_us = impl_->stats.avg_propagation_us.load(
        std::memory_order_relaxed);
    s.avg_esikf_update_ms    = impl_->stats.avg_update_ms.load(
        std::memory_order_relaxed);
    s.map_points             = impl_->ikd_tree.num_points;
    s.imu_queue_drops        = impl_->imu_ring.dropped();
    s.lidar_queue_drops      = impl_->cloud_ring.dropped();
    return s;
}

size_t AcmeSlamEngine::imuDropCount() const noexcept {
    return impl_->imu_ring.dropped();
}

size_t AcmeSlamEngine::cloudDropCount() const noexcept {
    return impl_->cloud_ring.dropped();
}

} // namespace thunderbird::odom
