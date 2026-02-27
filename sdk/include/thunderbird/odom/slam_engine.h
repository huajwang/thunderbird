// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — AcmeSlamEngine: Clean FAST-LIVO2 Estimator Wrapper
// ─────────────────────────────────────────────────────────────────────────────
//
// This class wraps the full FAST-LIVO2 SLAM pipeline (ESIKF + ikd-Tree +
// IMU preintegration + point deskewing) behind a minimal, non-blocking API.
//
// The public surface exposes ONLY SDK types from slam_types.h — no Eigen,
// no PCL, no ikd-Tree headers leak into user code.  All heavy dependencies
// are hidden behind the PImpl boundary in slam_engine.cpp.
//
// ═══════════════════════════════════════════════════════════════════════════
//  Thread Model — 4 threads, 3 lock-free queues
// ═══════════════════════════════════════════════════════════════════════════
//
//  ┌─────────────────────────────────────────────────────────────────────┐
//  │                                                                     │
//  │  Sensor threads (caller-owned)           AcmeSlamEngine threads     │
//  │  ─────────────────────────────           ──────────────────────     │
//  │                                                                     │
//  │  feedImu()                                                          │
//  │     │                                                               │
//  │     ▼                                                               │
//  │  ┌───────────────────┐       ┌────────────────────────────────┐    │
//  │  │ imu_ring_         │       │       Worker Thread            │    │
//  │  │ SPSC<ImuSample,   │ ────► │                                │    │
//  │  │      4096>        │       │  loop:                         │    │
//  │  └───────────────────┘       │    1. drain imu_ring_         │    │
//  │                              │    2. for each IMU sample:     │    │
//  │  feedPointCloud()            │         propagate()            │    │
//  │     │                        │         publish Pose6D         │    │
//  │     ▼                        │    3. if scan available:       │    │
//  │  ┌───────────────────┐       │         assemble IMU block    │    │
//  │  │ cloud_ring_       │ ────► │         deskew()              │    │
//  │  │ SPSC<CloudPtr, 16>│       │         esikf.update()        │    │
//  │  └───────────────────┘       │         ikdTree.insert()      │    │
//  │                              │         publish SlamOutput    │    │
//  │                              │    4. sleep_until next IMU     │    │
//  │                              └──────────┬─────────────────────┘    │
//  │                                         │                          │
//  │                                         ▼                          │
//  │  getLatestOutput()           ┌────────────────────────────┐        │
//  │     │                        │  output_ring_              │        │
//  │     ▼                        │  SPSC<SlamOutputPtr, 32>   │        │
//  │  ◄────────────────────────── │                            │        │
//  │                              └────────────────────────────┘        │
//  │                                                                     │
//  │  getLatestPose()             ┌────────────────────────────┐        │
//  │     │                        │  pose_ring_                │        │
//  │     ▼                        │  SPSC<Pose6DPtr, 256>      │        │
//  │  ◄────────────────────────── │  (200–1000 Hz IMU rate)    │        │
//  │                              └────────────────────────────┘        │
//  │                                                                     │
//  └─────────────────────────────────────────────────────────────────────┘
//
// Thread safety contract:
//
//   • feedImu()          — single-producer (IMU driver thread).   Non-blocking.
//   • feedPointCloud()   — single-producer (LiDAR driver thread). Non-blocking.
//   • getLatestOutput()  — single-consumer (user thread).         Non-blocking.
//   • getLatestPose()    — single-consumer (user thread).         Non-blocking.
//   • initialize()       — call once before feeding data.         Blocking (config I/O).
//   • reset()            — safe from user thread; sets atomic flag, worker drains.
//   • shutdown()         — call once; joins worker thread.        Blocking (join).
//
//   All lock-free SPSC RingBuffers are from ring_buffer.h — no mutex on the
//   hot path.  Cache-line padding avoids false sharing between head/tail.
//
// ═══════════════════════════════════════════════════════════════════════════
//  Memory Ownership Strategy
// ═══════════════════════════════════════════════════════════════════════════
//
//  Resource             Owner               Sharing Model
//  ─────────────────    ─────────────────    ──────────────────────────────
//  ImuSample            Copied on ingress    Value type (64 bytes, cheap).
//                       into imu_ring_.      Ring buffer owns the slots.
//
//  PointCloudFrame      shared_ptr<const>    Zero-copy: sensor thread moves
//                       moved into ring.     ownership into cloud_ring_.
//                                            Worker reads and may alias the
//                                            pointer into SlamOutput.
//
//  ESIKF state          Worker thread only   Never published.  Owned by Impl.
//  (Eigen matrices,     (private).           Deep-copied into Pose6D on each
//   ikd-Tree, etc.)                          propagation step.
//
//  Pose6D               shared_ptr<const>    Allocated by worker, published
//                       in pose_ring_.       via ring buffer.  User receives
//                                            a ref-counted pointer.
//
//  SlamOutput           shared_ptr<const>    Allocated by worker after ESIKF
//                       in output_ring_.     update.  Contains a Pose6D (by
//                                            value) and a shared_ptr to the
//                                            deskewed PointCloudFrame.
//
//  Deskewed cloud       shared_ptr<const>    Allocated by worker (new vector),
//                       inside SlamOutput.   shared between SlamOutput and
//                                            any user-held copies.
//
//  Config / Calibration Impl struct          Immutable after initialize().
//                       (private).           Worker reads without locking.
//
// No dynamic allocation on the hot path after initialize():
//   • IMU propagation:  copy 64-byte struct, matrix multiply (stack alloc).
//   • ESIKF update:     ikd-Tree search (uses node pool), Eigen (stack alloc).
//   • Output publish:   one make_shared<SlamOutput> per scan (10 Hz).
//   • Pose publish:     one make_shared<Pose6D> per IMU sample (200–1000 Hz).
//     (Amortizable with a pool allocator; initial version uses std allocator.)
//
// ═══════════════════════════════════════════════════════════════════════════
//  Deterministic Execution
// ═══════════════════════════════════════════════════════════════════════════
//
// The worker thread processes data strictly in timestamp order:
//
//   1. The SlamTimeSync layer (Step 4) ensures IMU samples arrive in
//      monotonic order and are boundary-interpolated to exact scan times.
//
//   2. The worker drains the IMU ring buffer ONE SAMPLE AT A TIME,
//      calling propagate() for each.  This is the same execution order
//      regardless of host scheduling jitter.
//
//   3. When a LiDAR scan is available and the IMU block covers it:
//        a. Assemble ScanMeasurement (via SlamTimeSync).
//        b. Deskew the point cloud using the propagated IMU trajectory.
//        c. Run ESIKF update (ikd-Tree nearest-neighbor + Kalman gain).
//        d. Insert deskewed points into the ikd-Tree.
//        e. Publish SlamOutput.
//
//   4. Given identical input sequences, the output is bit-identical
//      (modulo floating-point non-associativity across CPU arch).
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/odom/slam_types.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace thunderbird::odom {

// ═════════════════════════════════════════════════════════════════════════════
//  SlamEngineConfig — all tunable parameters for the SLAM pipeline
// ═════════════════════════════════════════════════════════════════════════════

/// IMU noise model (continuous-time spectral densities).
/// Used for ESIKF process noise Q-matrix construction.
struct ImuNoiseModel {
    double gyro_noise      = 1.0e-3;   ///< rad/s/√Hz   gyroscope white noise
    double accel_noise     = 1.0e-2;   ///< m/s²/√Hz    accelerometer white noise
    double gyro_bias_rw    = 1.0e-5;   ///< rad/s²/√Hz  gyroscope bias random walk
    double accel_bias_rw   = 1.0e-4;   ///< m/s³/√Hz    accelerometer bias random walk
};

/// LiDAR ↔ IMU rigid-body extrinsic calibration.
/// Transforms a point from LiDAR frame to IMU frame: p_imu = R * p_lidar + t
struct ExtrinsicCalibration {
    /// Rotation from LiDAR to IMU frame (Hamilton quaternion [w,x,y,z]).
    double rotation[4]    = {1.0, 0.0, 0.0, 0.0};

    /// Translation from LiDAR to IMU frame (metres).
    double translation[3] = {0.0, 0.0, 0.0};

    /// If true, the ESIKF will refine the extrinsic online.
    bool   refine_online  = false;
};

/// ikd-Tree and local map parameters.
struct MapConfig {
    double  voxel_resolution   = 0.3;    ///< voxel size for map downsampling (m)
    double  map_radius         = 100.0;  ///< local map bounding radius (m)
    size_t  max_map_points     = 500'000;///< hard ceiling on map points
    double  delete_ratio       = 0.1;    ///< fraction to trim when ceiling hit
    int     tree_rebalance_interval = 10;///< scans between ikd-Tree rebalances
};

/// ESIKF convergence parameters.
struct EsikfConfig {
    int     max_iterations     = 5;      ///< maximum Kalman iterations per update
    double  convergence_eps    = 1.0e-3; ///< convergence threshold (state delta norm)
    double  plane_noise_sigma  = 0.01;   ///< measurement noise σ for plane residuals (m)
    int     min_correspondences = 20;    ///< minimum matches to accept an update
    double  max_residual       = 0.5;    ///< reject update if residual exceeds this
};

/// Point deskewing (motion compensation) parameters.
struct DeskewConfig {
    bool    enable             = true;   ///< enable IMU-based point deskewing
    int     imu_integration_substeps = 1;///< sub-steps per point interval (1 = linear)
};

/// Gravity alignment / initialisation parameters.
struct InitConfig {
    double  init_gravity_duration_s = 1.0;  ///< seconds of static IMU data for gravity
    int     init_min_imu_samples    = 200;  ///< minimum samples before alignment
    double  init_gravity_tolerance  = 0.5;  ///< m/s² — max accel stddev to accept as static
};

/// Master configuration.
struct SlamEngineConfig {
    // ── Sensor model ────────────────────────────────────────────────────
    ImuNoiseModel          imu_noise;
    ExtrinsicCalibration   extrinsic;

    // ── Estimator ───────────────────────────────────────────────────────
    EsikfConfig            esikf;
    DeskewConfig           deskew;
    InitConfig             init;

    // ── Map ─────────────────────────────────────────────────────────────
    MapConfig              map;

    // ── Sensor rates (for buffer sizing / gap detection) ────────────────
    double imu_rate_hz     = 400.0;   ///< expected IMU rate
    double lidar_rate_hz   = 10.0;    ///< expected LiDAR rate

    // ── Time sync (forwarded to SlamTimeSync) ───────────────────────────
    bool   enable_drift_compensation = true;
    int64_t sort_window_ns           = 5'000'000;  // 5 ms

    // ── Queue capacities ────────────────────────────────────────────────
    /// IMU ingress ring buffer (power-of-two).
    static constexpr size_t kImuRingCapacity    = 4096;

    /// LiDAR ingress ring buffer (power-of-two).
    static constexpr size_t kCloudRingCapacity  = 16;

    /// SlamOutput output ring buffer (power-of-two).
    static constexpr size_t kOutputRingCapacity = 32;

    /// Pose output ring buffer (high-rate, power-of-two).
    static constexpr size_t kPoseRingCapacity   = 256;

    // ── Diagnostics ─────────────────────────────────────────────────────
    /// If true, the worker thread publishes a Pose6D for every IMU
    /// propagation step (200–1000 Hz).  If false, only corrected
    /// poses at LiDAR rate are published.
    bool publish_propagated_poses = true;
};

// ═════════════════════════════════════════════════════════════════════════════
//  AcmeSlamEngine — public API
// ═════════════════════════════════════════════════════════════════════════════
//
// Lifecycle:
//
//   AcmeSlamEngine engine;
//   engine.initialize(config);          // spawns worker thread
//
//   // from sensor threads:
//   engine.feedImu(sample);             // non-blocking, lock-free push
//   engine.feedPointCloud(cloud);       // non-blocking, lock-free push
//
//   // from user thread:
//   SlamOutput out;
//   if (engine.getLatestOutput(out))    // non-blocking, lock-free pop
//       processOutput(out);
//
//   engine.reset();                     // re-initialise (keeps config)
//   engine.shutdown();                  // joins worker, releases resources
//
// ─────────────────────────────────────────────────────────────────────────────

class AcmeSlamEngine {
public:
    // ── Construction / Destruction ──────────────────────────────────────

    AcmeSlamEngine();
    ~AcmeSlamEngine();

    // Non-copyable, non-movable (owns thread + PImpl).
    AcmeSlamEngine(const AcmeSlamEngine&) = delete;
    AcmeSlamEngine& operator=(const AcmeSlamEngine&) = delete;
    AcmeSlamEngine(AcmeSlamEngine&&) = delete;
    AcmeSlamEngine& operator=(AcmeSlamEngine&&) = delete;

    // ── Lifecycle ───────────────────────────────────────────────────────

    /// Initialise the SLAM pipeline with the given configuration.
    ///
    /// This is a blocking call that:
    ///   1. Validates the configuration.
    ///   2. Allocates internal buffers (ESIKF state, ikd-Tree, ring buffers).
    ///   3. Spawns the worker thread (which blocks until data arrives).
    ///
    /// @return true if initialisation succeeded.
    ///         false if already initialised, or config was invalid.
    ///
    /// Thread safety: must be called from one thread, before any feed*().
    bool initialize(const SlamEngineConfig& config);

    /// Reset the estimator to its initial state (Initializing).
    ///
    /// The config is preserved.  All queues are drained, the ikd-Tree is
    /// cleared, and the ESIKF state is zeroed.  The gravity alignment
    /// phase will re-execute on the next IMU data.
    ///
    /// Non-blocking from the caller's perspective: sets an atomic flag
    /// that the worker checks at the top of its loop.  The actual reset
    /// completes asynchronously within one worker iteration (< 1 ms).
    void reset();

    /// Shut down the SLAM pipeline.
    ///
    /// Signals the worker thread to exit and joins it.  After this call
    /// no further feed*() or get*() calls are valid.  The engine can be
    /// re-initialised with initialize().
    ///
    /// Blocking: waits for the worker thread to join.
    void shutdown();

    /// True if initialize() has been called and shutdown() has not.
    [[nodiscard]] bool isRunning() const noexcept;

    // ── Data Ingress (non-blocking, lock-free) ──────────────────────────

    /// Push an IMU sample into the ingress ring buffer.
    ///
    /// Single-producer contract: this must be called from at most one
    /// thread (the IMU driver thread).  If the buffer is full, the
    /// oldest sample is silently dropped (SPSC drop-oldest policy).
    ///
    /// @param sample   The IMU measurement (double-precision, 64 bytes).
    /// @param host_ns  Host arrival timestamp for clock drift estimation.
    ///                 Pass 0 to skip drift tracking.
    ///
    /// Latency: < 50 ns (single atomic store).
    void feedImu(const ImuSample& sample, int64_t host_ns = 0);

    /// Push a LiDAR point cloud into the ingress ring buffer.
    ///
    /// Single-producer contract: called from the LiDAR driver thread.
    /// Uses shared_ptr<const> for zero-copy transfer.
    ///
    /// @param cloud    Shared pointer to the point cloud frame.
    /// @param host_ns  Host arrival timestamp for drift estimation.
    ///
    /// Latency: < 50 ns (atomic store + ref-count bump).
    void feedPointCloud(std::shared_ptr<const PointCloudFrame> cloud,
                        int64_t host_ns = 0);

    // ── Data Egress (non-blocking, lock-free) ───────────────────────────

    /// Retrieve the latest SlamOutput (one per LiDAR scan, ~10 Hz).
    ///
    /// Returns true and fills `out` if a new output was available.
    /// Returns false if the output queue is empty (no new scan processed).
    ///
    /// The returned SlamOutput contains:
    ///   • Corrected Pose6D at the scan timestamp.
    ///   • Deskewed PointCloudFrame (shared_ptr, zero-copy).
    ///   • Map statistics and ESIKF diagnostics.
    ///
    /// If multiple outputs accumulated since the last call, only the
    /// most recent is returned and older ones are discarded.
    ///
    /// Single-consumer: must be called from one thread.
    ///
    /// @param[out] out  Receives the latest SlamOutput.
    /// @return true if a new output was available, false otherwise.
    bool getLatestOutput(SlamOutput& out);

    /// Retrieve all pending SlamOutputs since the last call.
    ///
    /// Unlike getLatestOutput() which discards older outputs, this
    /// drains the entire output queue.  Useful for recording or logging.
    ///
    /// @param[out] outputs  Receives all pending SlamOutputs (oldest first).
    /// @return Number of outputs returned.
    size_t drainOutputs(std::vector<SlamOutput>& outputs);

    /// Retrieve the latest propagated Pose6D (IMU-rate, 200–1000 Hz).
    ///
    /// Only available if config.publish_propagated_poses is true.
    ///
    /// @param[out] pose  Receives the latest Pose6D.
    /// @return true if a new pose was available, false otherwise.
    bool getLatestPose(Pose6D& pose);

    // ── Callbacks (alternative to polling) ──────────────────────────────

    /// Register a callback for every SlamOutput (~10 Hz).
    ///
    /// The callback fires on the worker thread immediately after
    /// each ESIKF update completes.  It must not block.
    ///
    /// @param cb  Callback function.  Pass nullptr to unregister.
    void onSlamOutput(SlamOutputCallback cb);

    /// Register a callback for every propagated pose (IMU rate).
    ///
    /// The callback fires on the worker thread after each IMU
    /// propagation step.  It must not block.
    ///
    /// @param cb  Callback function.  Pass nullptr to unregister.
    void onPose(PoseCallback cb);

    /// Register a callback for tracking status changes.
    ///
    /// Fires whenever the TrackingStatus transitions (e.g.
    /// Initializing → Converging → Tracking).
    ///
    /// @param cb  Callback function.  Pass nullptr to unregister.
    void onStatusChange(std::function<void(TrackingStatus)> cb);

    // ── Diagnostics ─────────────────────────────────────────────────────

    /// Current tracking status.
    [[nodiscard]] TrackingStatus status() const noexcept;

    /// Cumulative pipeline statistics.
    [[nodiscard]] OdometryStats stats() const noexcept;

    /// Runtime profiling snapshot — module-level CPU breakdown, latency
    /// percentiles, memory, and worker thread utilisation.
    ///
    /// Safe to call from any thread while the engine is running.
    /// Returns a copy of the profiler state at the time of the call.
    [[nodiscard]] ProfileSnapshot profileSnapshot() const noexcept;

    /// Number of IMU samples dropped due to ring buffer overflow.
    [[nodiscard]] size_t imuDropCount() const noexcept;

    /// Number of LiDAR scans dropped due to ring buffer overflow.
    [[nodiscard]] size_t cloudDropCount() const noexcept;

private:
    // ── PImpl — hides Eigen, ikd-Tree, ESIKF internals ─────────────────
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace thunderbird::odom
