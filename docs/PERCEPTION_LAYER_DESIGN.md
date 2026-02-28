# Thunderbird SDK — 3D Perception Layer Design

> **Status:** Implementation complete (Thunderbird SDK perception layer)  
> **Author:** Perception Architecture Team  
> **Date:** 2026-02-27  
> **Depends on:** `AcmeSlamEngine`, `RingBuffer<T,N>`, `slam_types.h`, drone/car YAML profiles  
> **Implementation scope:** CPU perception pipeline is fully implemented; GPU backends are currently stubbed and planned for a future phase.
---

## 1. Architecture Diagram

```
 ═══════════════════════════════════════════════════════════════════════════════
  Thread Model — SLAM + Perception (6 threads, 5 lock-free queues)
 ═══════════════════════════════════════════════════════════════════════════════

 ┌─────────────────────────────────────────────────────────────────────────────┐
 │                                                                             │
 │  SLAM Pipeline (existing)              Perception Pipeline (new)            │
 │  ────────────────────────              ──────────────────────────           │
 │                                                                             │
 │  AcmeSlamEngine                                                             │
 │  ┌──────────────────────────┐                                               │
 │  │  Worker Thread           │                                               │
 │  │  IMU propagate + ESIKF   │                                               │
 │  │  update + ikd-Tree       │                                               │
 │  └──────────┬───────────────┘                                               │
 │             │                                                               │
 │             │  SlamOutput (shared_ptr<const>)                               │
 │             │  ~10 Hz, contains:                                            │
 │             │    • Pose6D (corrected)                                       │
 │             │    • deskewed_cloud (shared_ptr<const PointCloudFrame>)       │
 │             │    • LocalMapInfo                                             │
 │             │                                                               │
 │             ▼                                                               │
 │  ┌──────────────────────────────────────────────────────────────────────┐   │
 │  │  perception_ring_  SPSC<shared_ptr<const SlamOutput>, 16>           │   │
 │  │  (zero-copy: just ref-count bump — SLAM worker never blocks)        │   │
 │  └──────────┬───────────────────────────────────────────────────────────┘   │
 │             │                                                               │
 │             ▼                                                               │
 │  ┌──────────────────────────────────────────────────────────────────────┐   │
 │  │  [T1] Preprocessor Thread (CPU)                                      │   │
 │  │                                                                      │   │
 │  │  1. Pop SlamOutput from perception_ring_                             │   │
 │  │  2. Voxel-downsample / ROI crop (profile-dependent)                  │   │
 │  │  3. Ground-plane segmentation (RANSAC or height threshold)           │   │
 │  │  4. Cluster non-ground into candidate regions (Euclidean cluster)    │   │
 │  │  5. Push DetectionInput into detection_ring_                         │   │
 │  │                                                                      │   │
 │  └──────────┬───────────────────────────────────────────────────────────┘   │
 │             │                                                               │
 │             ▼                                                               │
 │  ┌──────────────────────────────────────────────────────────────────────┐   │
 │  │  detection_ring_  SPSC<DetectionInput, 8>                            │   │
 │  └──────────┬───────────────────────────────────────────────────────────┘   │
 │             │                                                               │
 │             ▼                                                               │
 │  ┌──────────────────────────────────────────────────────────────────────┐   │
 │  │  [T2] Detector Thread (GPU or CPU, profile-selected)                 │   │
 │  │                                                                      │   │
 │  │  ┌──────────────────────────────────────────────────────────────┐    │   │
 │  │  │  GPU path (car mode):                                       │    │   │
 │  │  │    PointPillars / CenterPoint inference                     │    │   │
 │  │  │    TensorRT / ONNX Runtime (CUDA)                           │    │   │
 │  │  │    Classes: vehicle, pedestrian, cyclist, barrier            │    │   │
 │  │  ├──────────────────────────────────────────────────────────────┤    │   │
 │  │  │  CPU path (drone mode):                                     │    │   │
 │  │  │    Cluster-geometry classifier (L/W/H, density, eigenvals)  │    │   │
 │  │  │    Lightweight MLP or random forest                         │    │   │
 │  │  │    Classes: person, pole, wire, small_vehicle, unknown      │    │   │
 │  │  └──────────────────────────────────────────────────────────────┘    │   │
 │  │                                                                      │   │
 │  │  Output: RawDetections (3D bboxes + class + confidence)              │   │
 │  │  Push into tracking_ring_                                            │   │
 │  │                                                                      │   │
 │  └──────────┬───────────────────────────────────────────────────────────┘   │
 │             │                                                               │
 │             ▼                                                               │
 │  ┌──────────────────────────────────────────────────────────────────────┐   │
 │  │  tracking_ring_  SPSC<DetectionFrame, 16>                            │   │
 │  └──────────┬───────────────────────────────────────────────────────────┘   │
 │             │                                                               │
 │             ▼                                                               │
 │  ┌──────────────────────────────────────────────────────────────────────┐   │
 │  │  [T3] Tracker Thread (CPU)                                           │   │
 │  │                                                                      │   │
 │  │  1. Hungarian / greedy association (IoU-3D + Mahalanobis)            │   │
 │  │  2. Per-object Kalman filter (constant-velocity or CTRV)             │   │
 │  │  3. Track lifecycle: Tentative → Confirmed → Lost → Deleted          │   │
 │  │  4. Publish TrackedObjectList into output_ring_                      │   │
 │  │                                                                      │   │
 │  └──────────┬───────────────────────────────────────────────────────────┘   │
 │             │                                                               │
 │             ▼                                                               │
 │  ┌──────────────────────────────────────────────────────────────────────┐   │
 │  │  tracked_output_ring_  SPSC<shared_ptr<const TrackedObjectList>, 32>│   │
 │  └──────────┬───────────────────────────────────────────────────────────┘   │
 │             │                                                               │
 │             ▼                                                               │
 │  User thread:  getDetectedObjects() / onTrackedObjects(callback)           │
 │                                                                             │
 └─────────────────────────────────────────────────────────────────────────────┘
```

### Latency Budget (10 Hz input → 10 Hz output)

| Stage | Target (drone) | Target (car) | Thread |
|-------|----------------|--------------|--------|
| SLAM → perception_ring push | < 0.01 ms | < 0.01 ms | SLAM worker |
| Preprocessing | < 5 ms | < 10 ms | T1 (CPU) |
| Detection | < 15 ms | < 30 ms | T2 (GPU/CPU) |
| Tracking | < 2 ms | < 5 ms | T3 (CPU) |
| **End-to-end** | **< 22 ms** | **< 45 ms** | — |

At 10 Hz (100 ms budget), this leaves ample headroom for scheduling jitter.

---

## 2. Module Breakdown

### 2.1 `PerceptionEngine` (top-level orchestrator)

**File:** `sdk/include/thunderbird/perception/perception_engine.h`

Owns the three worker threads and all ring buffers. Follows the same PImpl
pattern as `AcmeSlamEngine` — no heavy headers leak into user code.

**Responsibilities:**
- Accept `SlamOutput` from the SLAM pipeline (callback or ring tap)
- Spawn T1/T2/T3 threads on `start()`
- Expose pull API (`getDetectedObjects()`) and callback API (`onTrackedObjects()`)
- Lifecycle: `initialize()` → `start()` → `stop()` → `shutdown()`
- Profile-aware: loads drone or car config to select detector backend + parameters

### 2.2 `PointCloudPreprocessor` (T1 — CPU)

**File:** `sdk/include/thunderbird/perception/preprocessor.h`

Stateless transform applied per-frame. All CPU, no GPU dependency.

| Sub-step | Drone mode | Car mode |
|----------|-----------|----------|
| Voxel filter | 0.1 m | 0.2 m |
| ROI crop | 30 m radius, ±15 m height | 80 m radius, ±5 m height |
| Ground removal | Height threshold (fast) | RANSAC plane fit |
| Clustering | Euclidean (eps=0.5 m, min=10) | Euclidean (eps=0.8 m, min=20) |

**Output:** `DetectionInput` — ground-removed clusters + ego pose + timestamp.

### 2.3 `ObjectDetector` — abstract interface (T2)

**File:** `sdk/include/thunderbird/perception/object_detector.h`

Pure virtual base. Two concrete implementations selected at runtime:

#### 2.3.1 `GpuCenterPointDetector` (car mode — primary)

- CenterPoint anchor-free architecture (voxel backbone + center heatmap head)
- Runs on GPU via TensorRT FP16 (requires `spconv` TensorRT plugin)
- Input: voxelized point cloud → 3D sparse conv → BEV features
- Output: oriented 3D bounding boxes with continuous yaw + class + score
- 18–25 Hz on Jetson AGX Orin, ~400 MB GPU memory
- Best rotation estimation for CTRV tracker motion model

#### 2.3.2 `GpuPillarDetector` (car mode — fallback, drone mode — primary)

- PointPillars architecture (pillar pseudo-image + 2D backbone)
- Runs on GPU via TensorRT FP16 (all standard ops, no plugins)
- Input: pillar-scattered pseudo-image (C×H×W)
- Output: anchor-based 3D bounding boxes + class + score
- 20–30 Hz on Jetson Orin Nano, ~200 MB GPU memory
- Drone variant: re-anchored for small objects (person, pole), tighter ROI

#### 2.3.3 `CpuClusterDetector` (drone mode — CPU-only fallback)

- Per-cluster geometric features: L/W/H, point density, eigenvalue ratios
- Lightweight MLP or rule-based classifier (< 1 ms per frame)
- No GPU required — critical for embedded drone payloads without GPU (RPi CM5)

### 2.4 `MultiObjectTracker` (T3 — CPU)

**File:** `sdk/include/thunderbird/perception/multi_object_tracker.h`

Maintains persistent track state across frames.

| Component | Description |
|-----------|-------------|
| Association | Hungarian algorithm on cost matrix (IoU-3D + center distance) |
| State filter | Per-track EKF: state = [x, y, z, θ, vx, vy, vz] (yaw_rate ω derived from θ, not part of state) |
| Motion model | Constant velocity (drone) / CTRV (car) — profile-selected |
| Track lifecycle | `Tentative(N=2)` → `Confirmed(N=3)` → `Coasting(miss≤5)` → `Deleted` |
| ID assignment | Monotonic uint64 track IDs, never reused within a session |

### 2.5 `PerceptionConfig` (YAML-driven, profile-aware)

**File:** `sdk/include/thunderbird/perception/perception_config.h`

```yaml
# Example: drone perception config section appended to drone.yaml
perception:
  enable: true
  mode: drone                   # "drone" | "car"

  preprocessor:
    voxel_size: 0.1
    roi_radius: 30.0
    roi_z_min: -15.0
    roi_z_max: 15.0
    ground_removal: height      # "height" | "ransac"
    ground_height: -0.3
    cluster_eps: 0.5
    cluster_min_points: 10

  detector:
    backend: cpu_cluster        # "cpu_cluster" | "gpu_pillar"
    # GPU-specific (ignored in cpu_cluster mode):
    model_path: ""
    confidence_threshold: 0.3
    nms_iou_threshold: 0.5

  tracker:
    motion_model: constant_velocity   # "constant_velocity" | "ctrv"
    association_metric: iou_3d        # "iou_3d" | "center_distance"
    confirm_hits: 3
    max_coast_frames: 5
    process_noise_pos: 0.5
    process_noise_vel: 1.0
    measurement_noise: 0.3
```

---

## 3. Interface Definitions

### 3.1 Public Data Types

All types follow existing SDK conventions: Eigen-free, standard-layout where
possible, `shared_ptr<const T>` for zero-copy publishing.

```cpp
namespace thunderbird::perception {

// ─── Object class taxonomy ──────────────────────────────────────────────────

enum class ObjectClass : uint8_t {
    Unknown     = 0,
    Vehicle     = 1,
    Pedestrian  = 2,
    Cyclist     = 3,
    Barrier     = 4,
    // Drone-specific
    Person      = 10,
    Pole        = 11,
    Wire        = 12,
    SmallVehicle= 13,
};

// ─── 3D Oriented Bounding Box ───────────────────────────────────────────────

struct BBox3D {
    double center[3];        // x, y, z  in world frame
    double extent[3];        // length, width, height (metres)
    double yaw;              // heading angle (radians, world Z-up)
};

// ─── Single Detection (per-frame, before tracking) ─────────────────────────

struct Detection3D {
    BBox3D      bbox;
    ObjectClass label;
    float       confidence;   // [0, 1]
    uint32_t    cluster_id;   // cross-reference to preprocessor cluster
    uint32_t    num_points;   // points inside this detection
};

// ─── Detection frame (output of detector, input to tracker) ────────────────

struct DetectionFrame {
    int64_t                   timestamp_ns;
    odom::Pose6D              ego_pose;       // world-frame reference
    std::vector<Detection3D>  detections;
};

// ─── Track state ────────────────────────────────────────────────────────────

enum class TrackState : uint8_t {
    Tentative  = 0,
    Confirmed  = 1,
    Coasting   = 2,  // missed detection, predicting
};

// ─── Single Tracked Object ─────────────────────────────────────────────────

struct TrackedObject {
    uint64_t    track_id;            // unique, monotonic, never reused
    TrackState  state;
    ObjectClass label;
    float       confidence;          // smoothed confidence

    BBox3D      bbox;                // filtered position + extent
    double      velocity[3];         // vx, vy, vz  (m/s, world frame)
    double      yaw_rate;            // rad/s
    int         age_frames;          // frames since first detection
    int         hits;                // total matched detections
    int         consecutive_misses;  // current coast streak

    // 7×7 covariance (x, y, z, yaw, vx, vy, vz) row-major
    double      covariance[49];
};

// ─── Tracked Object List (published per frame) ─────────────────────────────

struct TrackedObjectList {
    int64_t                       timestamp_ns;
    odom::Pose6D                  ego_pose;
    std::vector<TrackedObject>    objects;
    uint32_t                      frame_sequence;

    // Diagnostics
    double   preprocess_ms;
    double   detection_ms;
    double   tracking_ms;
    uint32_t input_points;
    uint32_t filtered_points;
    uint32_t num_clusters;
};

// ─── Callbacks ──────────────────────────────────────────────────────────────

using TrackedObjectCallback = std::function<void(std::shared_ptr<const TrackedObjectList>)>;
using DetectionCallback     = std::function<void(std::shared_ptr<const DetectionFrame>)>;

} // namespace thunderbird::perception
```

### 3.2 `PerceptionEngine` Public API

```cpp
namespace thunderbird::perception {

class PerceptionEngine {
public:
    PerceptionEngine();
    ~PerceptionEngine();

    // Non-copyable, non-movable (owns threads + PImpl)
    PerceptionEngine(const PerceptionEngine&) = delete;
    PerceptionEngine& operator=(const PerceptionEngine&) = delete;

    // ── Lifecycle ───────────────────────────────────────────────────────
    bool initialize(const PerceptionConfig& config);
    void start();
    void stop();
    void shutdown();

    // ── SLAM integration (called from SLAM output callback) ─────────────
    /// Feed a SlamOutput into the perception pipeline.
    /// Non-blocking, lock-free push into perception_ring_.
    /// Drops oldest if ring is full (perception slower than SLAM).
    void feedSlamOutput(std::shared_ptr<const odom::SlamOutput> output);

    // ── Pull API (user thread, non-blocking) ────────────────────────────
    bool getDetectedObjects(std::shared_ptr<const TrackedObjectList>& out);
    bool getLatestDetections(std::shared_ptr<const DetectionFrame>& out);

    // ── Callback API ────────────────────────────────────────────────────
    void onTrackedObjects(TrackedObjectCallback cb);
    void onDetections(DetectionCallback cb);

    // ── Diagnostics ─────────────────────────────────────────────────────
    struct PerceptionStats {
        uint64_t frames_processed;
        uint64_t frames_dropped;
        double   avg_preprocess_ms;
        double   avg_detection_ms;
        double   avg_tracking_ms;
        double   avg_total_ms;
        uint32_t active_tracks;
    };
    PerceptionStats stats() const;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace thunderbird::perception
```

### 3.3 `ObjectDetector` Abstract Interface

```cpp
namespace thunderbird::perception {

struct DetectionInput {
    int64_t                  timestamp_ns;
    odom::Pose6D             ego_pose;
    // Ground-removed, clustered point cloud
    std::shared_ptr<const odom::PointCloudFrame> filtered_cloud;
    // Pre-computed cluster indices [point_idx → cluster_id]
    std::vector<uint32_t>    cluster_labels;
    uint32_t                 num_clusters;
    // Ground plane coefficients (ax + by + cz + d = 0)
    double                   ground_plane[4];
};

class ObjectDetector {
public:
    virtual ~ObjectDetector() = default;

    /// One-time initialization (load model, allocate GPU memory, etc.)
    virtual bool initialize(const PerceptionConfig& config) = 0;

    /// Synchronous detection on the caller's thread (T2).
    /// Must be re-entrant but NOT thread-safe (called from single thread).
    virtual DetectionFrame detect(const DetectionInput& input) = 0;

    /// Name of the backend (for logging / diagnostics).
    virtual const char* name() const = 0;

    /// Factory: create the correct detector for the profile.
    static std::unique_ptr<ObjectDetector> create(const PerceptionConfig& config);
};

} // namespace thunderbird::perception
```

### 3.4 `PointCloudPreprocessor` Interface

```cpp
namespace thunderbird::perception {

class PointCloudPreprocessor {
public:
    explicit PointCloudPreprocessor(const PerceptionConfig& config);

    /// Process a raw deskewed cloud into a DetectionInput.
    /// Pure function — no internal state between frames.
    DetectionInput process(
        std::shared_ptr<const odom::PointCloudFrame> cloud,
        const odom::Pose6D& ego_pose,
        int64_t timestamp_ns);

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;  // hides PCL / nanoflann dependency
};

} // namespace thunderbird::perception
```

### 3.5 `MultiObjectTracker` Interface

```cpp
namespace thunderbird::perception {

class MultiObjectTracker {
public:
    explicit MultiObjectTracker(const PerceptionConfig& config);

    /// Update tracks with new detections. Returns stable tracked objects.
    /// Stateful — must be called from a single thread (T3).
    TrackedObjectList update(const DetectionFrame& detections);

    /// Reset all tracks (e.g., after SLAM re-initialization).
    void reset();

    uint32_t activeTrackCount() const;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace thunderbird::perception
```

---

## 4. Threading Model

### 4.1 Thread Inventory

| Thread | Name | Affinity | Purpose | Blocking? |
|--------|------|----------|---------|-----------|
| Existing: SLAM worker | `slam_worker` | CPU core 0-1 | ESIKF + ikd-Tree | Never blocks on perception |
| T1 | `percept_preproc` | CPU core 2 | Voxel filter, ground seg, clustering | Blocks on detection_ring_ full (backpressure) |
| T2 | `percept_detect` | GPU + CPU core 3 | Neural net inference or cluster classify | Blocks on GPU kernel completion |
| T3 | `percept_track` | CPU core 2 | Association + Kalman + lifecycle | Never blocks downstream |

### 4.2 Decoupling from SLAM — The Critical Boundary

The SLAM pipeline must **never** block on perception. This is achieved by:

1. **Async ring-buffer tap:** The SLAM worker's `onSlamOutput` callback does a
   single `perception_ring_.push()` — a lock-free write that completes in
   < 100 ns. If perception falls behind, the ring overwrites the oldest
   unprocessed frame (drop policy, not block policy).

2. **Zero-copy handoff:** `SlamOutput` is already published as
   `shared_ptr<const>`. Pushing into `perception_ring_` is just a pointer copy
   + atomic ref-count increment. No point cloud data is copied.

3. **Independent wake cycle:** T1 runs its own `condition_variable` or busy-poll
   loop on `perception_ring_`, completely decoupled from the SLAM worker's
   wake/sleep cycle.

4. **Graceful degradation:** If perception can't keep up with 10 Hz input,
   it drops frames and logs a counter. SLAM accuracy is never affected.

```
 SLAM Worker Thread                    Perception T1 Thread
 ──────────────────                    ─────────────────────
 esikf.update()                        loop:
 publish SlamOutput                      if perception_ring_.pop():
   │                                        preprocess()
   │  push(shared_ptr)   ← LOCK-FREE →      push detection_ring_
   │  ~100 ns                              else:
   │                                        wait_for(500 µs)
   ▼
 continue SLAM loop
 (never waits on perception)
```

### 4.3 Backpressure and Drop Policy

| Ring buffer | Capacity | Drop policy | Rationale |
|-------------|----------|-------------|-----------|
| `perception_ring_` | 16 | Drop oldest | SLAM must never stall; occasional frame skip is acceptable |
| `detection_ring_` | 8 | Drop oldest | If detector is slow, skip stale preprocessed data |
| `tracking_ring_` | 16 | Drop oldest | Detector output to tracker; tracker is fast, rarely drops |
| `tracked_output_ring_` | 32 | Drop oldest | User consumption; generous buffer for bursty reads |

---

## 5. GPU vs CPU Separation

### 5.1 Execution Domain Map

```
 ┌─────────────────────────────────────────────────────────────┐
 │                        CPU-ONLY DOMAIN                      │
 │                                                             │
 │  ┌───────────┐    ┌──────────────┐    ┌──────────────────┐ │
 │  │ Preproc   │    │ Tracker      │    │ Output publish   │ │
 │  │ T1        │    │ T3           │    │ (ring push)      │ │
 │  │           │    │              │    │                  │ │
 │  │ • voxel   │    │ • Hungarian  │    │ • callback fire  │ │
 │  │ • ground  │    │ • Kalman     │    │ • pull API       │ │
 │  │ • cluster │    │ • lifecycle  │    │                  │ │
 │  └───────────┘    └──────────────┘    └──────────────────┘ │
 │                                                             │
 ├─────────────────────────────────────────────────────────────┤
 │           GPU DOMAIN (car mode only, T2)                    │
 │                                                             │
 │  ┌──────────────────────────────────────────────────────┐  │
 │  │  Detector (GpuPillarDetector)                        │  │
 │  │                                                      │  │
 │  │  CPU side:          GPU side:                        │  │
 │  │  • pillar scatter   • backbone CNN (TensorRT)        │  │
 │  │  • NMS              • detection head                 │  │
 │  │  • result decode    • BEV feature extraction         │  │
 │  │                                                      │  │
 │  │  Memory:                                             │  │
 │  │  • Pre-allocate GPU buffers at initialize()          │  │
 │  │  • Pinned host memory for async H2D / D2H            │  │
 │  │  • CUDA stream per detector instance                 │  │
 │  │  • No cudaMalloc on hot path                         │  │
 │  └──────────────────────────────────────────────────────┘  │
 │                                                             │
 ├─────────────────────────────────────────────────────────────┤
 │           CPU DOMAIN (drone mode, T2)                       │
 │                                                             │
 │  ┌──────────────────────────────────────────────────────┐  │
 │  │  Detector (CpuClusterDetector)                       │  │
 │  │                                                      │  │
 │  │  • cluster PCA → eigenvalue ratios                   │  │
 │  │  • geometric features (L/W/H, density)               │  │
 │  │  • rule-based or lightweight MLP classification      │  │
 │  │  • No GPU, no CUDA dependency                        │  │
 │  │  • ~1 ms per frame on ARM Cortex-A78 (Jetson)       │  │
 │  └──────────────────────────────────────────────────────┘  │
 │                                                             │
 └─────────────────────────────────────────────────────────────┘
```

### 5.2 Build-time GPU Isolation

```cmake
option(THUNDERBIRD_ENABLE_GPU_PERCEPTION "Build GPU detector backend (requires CUDA + TensorRT)" OFF)

# Core perception (always built, CPU-only)
add_library(thunderbird_perception
    src/perception/perception_engine.cpp
    src/perception/preprocessor.cpp
    src/perception/multi_object_tracker.cpp
    src/perception/cpu_cluster_detector.cpp
)

# GPU detector (optional, guarded by CUDA availability)
if(THUNDERBIRD_ENABLE_GPU_PERCEPTION)
    find_package(CUDA REQUIRED)
    find_package(TensorRT REQUIRED)
    target_sources(thunderbird_perception PRIVATE
        src/perception/gpu_pillar_detector.cpp
        src/perception/gpu_pillar_detector.cu
    )
    target_compile_definitions(thunderbird_perception PUBLIC THUNDERBIRD_HAS_GPU_PERCEPTION)
    target_link_libraries(thunderbird_perception PRIVATE ${TensorRT_LIBRARIES} ${CUDA_LIBRARIES})
endif()
```

This ensures the core SDK compiles on any platform (ARM drone, x86 CI) without
CUDA installed. The `ObjectDetector::create()` factory returns `CpuClusterDetector`
when GPU is unavailable or when `config.detector.backend == "cpu_cluster"`.

---

## 6. File Layout

```
sdk/
├── include/thunderbird/perception/
│   ├── perception_engine.h          # Top-level orchestrator (public API)
│   ├── perception_config.h          # PerceptionConfig struct
│   ├── perception_types.h           # BBox3D, Detection3D, TrackedObject, etc.
│   ├── object_detector.h            # ObjectDetector abstract interface + factory
│   ├── preprocessor.h               # PointCloudPreprocessor interface
│   └── multi_object_tracker.h       # MultiObjectTracker interface
│
├── src/perception/
│   ├── perception_engine.cpp         # PImpl: ring buffers, threads, lifecycle
│   ├── preprocessor.cpp              # Voxel filter, ground seg, clustering
│   ├── cpu_cluster_detector.cpp      # Drone-mode: geometry-based classifier
│   ├── gpu_pillar_detector.cpp       # Car-mode: TensorRT PointPillars (optional)
│   ├── gpu_pillar_detector.cu        # CUDA kernels for pillar scatter (optional)
│   └── multi_object_tracker.cpp      # Hungarian + Kalman + lifecycle
│
slamd/config/
│   ├── drone.yaml                    # Add `perception:` section
│   └── car.yaml                      # Add `perception:` section
│
tests/
│   ├── test_perception_engine.cpp
│   ├── test_preprocessor.cpp
│   ├── test_object_detector.cpp
│   └── test_multi_object_tracker.cpp
```

---

## 7. Integration with Existing `SlamDaemon`

The `SlamDaemon` (in `slamd/`) already wires `DeviceManager → AcmeSlamEngine → IPC`.
The perception layer plugs in as a parallel consumer of `SlamOutput`:

```
 SlamDaemon::start() {
     // ... existing wiring ...
     slam_engine_.onSlamOutput([this](auto output) {
         ipc_publisher_.publish(output);          // existing
         perception_engine_.feedSlamOutput(output); // NEW — non-blocking tap
     });
     perception_engine_.initialize(config_.perception);
     perception_engine_.start();
 }
```

The perception output can be:
- Published over IPC (new shared-memory channel for tracked objects)
- Forwarded to ROS2 bridge as `visualization_msgs/MarkerArray` + custom `TrackedObjectArray`
- Consumed via the SDK pull/callback API

---

## 8. Drone vs Car Mode Summary

| Aspect | Drone mode | Car mode |
|--------|-----------|----------|
| **Detector** | `GpuPillarDetector` (GPU) / `CpuClusterDetector` (no GPU) | `GpuCenterPointDetector` (primary) / `GpuPillarDetector` (fallback) |
| **GPU required** | Preferred (Orin Nano) / No (RPi CM5) | Yes (TensorRT, Orin AGX) |
| **Classes** | person, pole, wire, small_vehicle | vehicle, pedestrian, cyclist, barrier |
| **Voxel size** | 0.1 m | 0.2 m |
| **ROI radius** | 30 m | 80 m |
| **Ground removal** | Height threshold | RANSAC plane fit |
| **Motion model** | Constant velocity | CTRV (constant turn-rate, velocity) |
| **Latency target** | < 22 ms | < 45 ms |
| **Platform** | Jetson Orin Nano, RPi CM5 | Jetson Orin, x86 + discrete GPU |

---

## 9. 3D Detection Model Selection

### 9.1 Candidate Models

| Property | PointPillars | SECOND | CenterPoint | PV-RCNN |
|----------|-------------|--------|-------------|---------|
| **Year / venue** | 2019 / CVPR | 2018 / Sensors | 2021 / CVPR | 2020 / CVPR |
| **Representation** | Pillar-based (pseudo-image) | Sparse 3D voxel convolutions | Voxel backbone + center heatmap | Voxel + raw point features |
| **Stage** | Single-stage | Single-stage | Single-stage (anchor-free) | Two-stage |
| **Core operators** | 2D conv (PointNet per pillar → BEV) | 3D sparse conv (submanifold) | 3D sparse conv + 2D BEV head | 3D sparse conv + PointNet + RoI grid pool |
| **Strength** | Speed; trivially TensorRT-able | Good accuracy/speed tradeoff | Best anchor-free accuracy; rotation-aware | Highest accuracy on KITTI |
| **Weakness** | Height information compressed | Moderate complexity | Heavier than PointPillars | Very heavy inference; two-stage overhead |

### 9.2 Jetson Orin Class Performance Profile

Reference platform: **NVIDIA Jetson AGX Orin 64 GB** (275 TOPS INT8, 2048 CUDA cores, Ampere) and **Jetson Orin Nano 8 GB** (40 TOPS INT8, 1024 CUDA cores).

All numbers below assume **TensorRT FP16** engine, single CUDA stream, KITTI-scale input (~120 k points, 80 m range), and include full pipeline (voxelization + backbone + head + NMS):

| Metric | PointPillars | SECOND | CenterPoint | PV-RCNN |
|--------|-------------|--------|-------------|---------|
| **Orin AGX — FPS** | 45–62 Hz | 22–30 Hz | 18–25 Hz | 5–8 Hz |
| **Orin Nano — FPS** | 20–30 Hz | 10–14 Hz | 8–12 Hz | 2–4 Hz |
| **GPU memory (FP16)** | 180–250 MB | 300–450 MB | 350–500 MB | 600–900 MB |
| **Host RAM overhead** | ~50 MB | ~80 MB | ~90 MB | ~150 MB |
| **Power draw (module)** | 8–12 W | 12–18 W | 15–22 W | 22–30 W |
| **TensorRT export** | Trivial (all standard ops) | Moderate (sparse conv plugin) | Moderate (sparse conv + deform) | Hard (PointNet SA + RoI pool) |

### 9.3 Accuracy Benchmark (KITTI val, Car class, 3D AP @ IoU 0.7)

| Difficulty | PointPillars | SECOND | CenterPoint | PV-RCNN |
|------------|-------------|--------|-------------|---------|
| Easy | 82.6 | 84.7 | 85.2 | **90.3** |
| Moderate | 74.3 | 76.5 | 77.8 | **81.4** |
| Hard | 68.1 | 69.8 | 73.1 | **78.6** |

PV-RCNN leads in accuracy by a significant margin, but at 5–8 Hz on Orin AGX it
**fails the 10 Hz hard real-time requirement** and its power draw is prohibitive
for edge deployment.

### 9.4 Decision Matrix — Weighted Scoring

Weights reflect Thunderbird's constraints: real-time (10–20 Hz), edge GPU,
moderate power, production-grade deployment.

| Criterion (weight) | PointPillars | SECOND | CenterPoint | PV-RCNN |
|--------------------|-------------|--------|-------------|---------|
| **Inference speed (30%)** | ★★★★★ | ★★★☆☆ | ★★★☆☆ | ★☆☆☆☆ |
| **Detection accuracy (25%)** | ★★★☆☆ | ★★★★☆ | ★★★★☆ | ★★★★★ |
| **Memory footprint (15%)** | ★★★★★ | ★★★☆☆ | ★★★☆☆ | ★★☆☆☆ |
| **Power efficiency (10%)** | ★★★★★ | ★★★☆☆ | ★★★☆☆ | ★☆☆☆☆ |
| **TensorRT integration (10%)** | ★★★★★ | ★★★☆☆ | ★★★☆☆ | ★★☆☆☆ |
| **Rotation estimation (10%)** | ★★☆☆☆ | ★★★☆☆ | ★★★★★ | ★★★★★ |
| **Weighted total** | **4.05** | **3.25** | **3.45** | **2.75** |

### 9.5 Recommendations

#### Autonomous Vehicle (Car Mode)

**Primary: CenterPoint (TensorRT FP16)**

| Property | Value |
|----------|-------|
| Target platform | Jetson AGX Orin |
| Expected FPS | 18–25 Hz (meets 10 Hz target with 80–150% headroom) |
| GPU memory | ~400 MB reserved at init |
| Power | ~18 W (module-level) |
| Training data | nuScenes (10 classes) or KITTI (3 classes) pre-trained, fine-tune on target domain |
| Key advantage | Anchor-free center heatmap produces accurate **oriented** bounding boxes with continuous yaw — critical for downstream tracking with CTRV motion model |

CenterPoint is preferred over PointPillars for car mode because:
1. **Rotation-aware output:** The center-heatmap head directly regresses yaw without anchor
   discretisation artifacts. This feeds cleaner orientation into the `MultiObjectTracker`'s CTRV model.
2. **Sufficient speed:** 18–25 Hz on Orin AGX exceeds our 10 Hz SLAM-output rate.
   Even on an Orin NX (mid-tier), it achieves 12–16 Hz.
3. **Better pedestrian/cyclist recall:** Anchor-free design handles small, thin objects
   better than PointPillars' fixed anchor grid.

**Fallback: PointPillars** if deployed on Orin NX/Nano-class hardware where CenterPoint
drops below 10 Hz.

```
PerceptionConfig (car.yaml):
  detector:
    backend: gpu_centerpoint       # primary
    fallback_backend: gpu_pillar   # auto-switch if FPS < 10
    model_path: /opt/thunderbird/models/centerpoint_kitti_fp16.engine
    confidence_threshold: 0.35
    nms_iou_threshold: 0.5
    max_detections: 200
```

#### Drone Environment (Drone Mode)

**Primary: PointPillars (TensorRT FP16) — when Orin Nano GPU is available**
**Fallback: CpuClusterDetector — when no GPU or power-constrained**

| Property | PointPillars (drone) | CpuClusterDetector |
|----------|---------------------|-------------------|
| Target platform | Jetson Orin Nano | Any ARM (RPi CM5, no GPU) |
| Expected FPS | 20–30 Hz | 50–100 Hz |
| GPU memory | ~200 MB | 0 |
| Host RAM | ~50 MB | ~20 MB |
| Power | ~10 W | ~3 W |
| Accuracy | Good (re-trained on small objects) | Moderate (geometry-only, no learned features) |

Drone-mode PointPillars requires modifications vs. the standard KITTI model:

| Adaptation | Standard (car) | Drone-adapted |
|-----------|---------------|---------------|
| Point cloud range | [0, 80] × [-40, 40] × [-3, 1] m | [-30, 30] × [-30, 30] × [-15, 15] m |
| Pillar size | 0.16 × 0.16 m | 0.10 × 0.10 m |
| Max pillars | 12,000 | 8,000 (fewer points from compact LiDAR) |
| Max points/pillar | 32 | 20 |
| Classes | vehicle, pedestrian, cyclist | person, pole, small_vehicle |
| Anchor sizes | car: 3.9×1.6×1.56 m | person: 0.6×0.6×1.7 m, pole: 0.3×0.3×2.5 m |

PointPillars wins for drone because:
1. **Fastest inference:** 20–30 Hz on Orin Nano is already 2–3× our 10 Hz input rate.
2. **Smallest memory:** 200 MB leaves headroom for SLAM + sensor drivers on 8 GB.
3. **Trivial TensorRT export:** No sparse conv plugins needed — just standard 2D convolutions.
   This dramatically reduces deployment risk on Jetson.
4. **Simpler voxelization:** Pillar scatter is a single CUDA kernel; no submanifold
   sparse tensor bookkeeping.

CenterPoint and SECOND are **not recommended** for drone because their 3D sparse
convolution dependency (`spconv`) requires a TensorRT plugin that is fragile on
JetPack updates and adds 100+ MB to the engine.

PV-RCNN is **eliminated** for both platforms: too slow, too much memory, too hard to export.

### 9.6 Model Comparison — Architecture Diagrams

```
 PointPillars (drone primary)
 ────────────────────────────
 Raw Points → Pillar Scatter (CUDA) → PointNet per pillar → Pseudo-image (C×H×W)
           → 2D Backbone (ResNet-lite) → BEV feature map
           → SSD-style detection head → 3D boxes + class + score
           → NMS

 CenterPoint (car primary)
 ──────────────────────────
 Raw Points → Voxelization → 3D Sparse Conv Backbone (spconv)
           → BEV Scatter → 2D Backbone (ResNet)
           → Center Heatmap Head:
               ├─ heatmap (class centers)
               ├─ offset (sub-voxel center refinement)
               ├─ height (z center)
               ├─ size (l, w, h)
               ├─ rotation (sin θ, cos θ)
               └─ velocity (vx, vy) [optional]
           → Peak extraction → 3D boxes
```

### 9.7 Integration Plan with `ObjectDetector` Interface

Both models implement the existing `ObjectDetector` abstract interface from the
perception design. The factory selects based on config:

```cpp
std::unique_ptr<ObjectDetector> ObjectDetector::create(const PerceptionConfig& config) {
    const auto& backend = config.detector.backend;

    if (backend == "gpu_centerpoint") {
#ifdef THUNDERBIRD_HAS_GPU_PERCEPTION
        return std::make_unique<GpuCenterPointDetector>();
#else
        LOG_WARN("CenterPoint requested but GPU perception not compiled; falling back");
#endif
    }

    if (backend == "gpu_pillar") {
#ifdef THUNDERBIRD_HAS_GPU_PERCEPTION
        return std::make_unique<GpuPillarDetector>();
#else
        LOG_WARN("PointPillars requested but GPU perception not compiled; falling back");
#endif
    }

    // Default: CPU cluster classifier (always available)
    return std::make_unique<CpuClusterDetector>();
}
```

Updated file layout adds one source file:

```
sdk/src/perception/
├── gpu_pillar_detector.cpp          # PointPillars TensorRT wrapper
├── gpu_pillar_detector.cu           # Pillar scatter CUDA kernel
├── gpu_centerpoint_detector.cpp     # CenterPoint TensorRT wrapper  ← NEW
├── gpu_centerpoint_detector.cu      # Voxelization + scatter kernel  ← NEW
├── cpu_cluster_detector.cpp         # Geometry-based (no GPU)
└── ...
```

### 9.8 Summary Table

| | **Car Mode** | **Drone Mode** |
|---|---|---|
| **Model** | CenterPoint | PointPillars |
| **Fallback** | PointPillars | CpuClusterDetector |
| **Platform** | Orin AGX (64 GB) | Orin Nano (8 GB) |
| **Expected FPS** | 18–25 Hz | 20–30 Hz |
| **GPU VRAM** | ~400 MB | ~200 MB |
| **Power** | ~18 W | ~10 W |
| **TensorRT complexity** | Moderate (spconv plugin) | Low (standard ops) |
| **Accuracy (KITTI mod.)** | 77.8% AP | 74.3% AP (re-anchored) |
| **Yaw estimation** | Excellent (heatmap regression) | Adequate (anchor-based) |
| **Why this model** | Best oriented-bbox accuracy that still meets 10 Hz | Fastest, smallest, easiest TensorRT path |
| **Why not PV-RCNN** | 5–8 Hz, 600+ MB, hard export | Same, worse on Orin Nano |
| **Why not SECOND** | Same spconv cost as CenterPoint but lower accuracy | spconv plugin fragile on JetPack |

---

## 10. Open Questions for Implementation Phase

1. **Point cloud format:** Should `DetectionInput` carry the full `PointCloudFrame` or a
   downsampled representation (e.g., voxel centers + occupancy counts)?
2. **Model training data:** CenterPoint — nuScenes pre-trained, fine-tune on KITTI or
   target domain. PointPillars (drone) — requires custom dataset with small-object
   annotations. Which data collection strategy?
3. **Coordinate frame:** Detections in world frame or ego frame? Tracker state in
   which frame? Proposal: detect in ego frame, track in world frame, output both.
4. **GPU memory budget:** CenterPoint ~400 MB, PointPillars ~200 MB pre-allocated
   at init. Confirm Orin AGX/Nano headroom after SLAM + driver allocations.
5. **ROS2 message types:** Use standard `vision_msgs/Detection3DArray` or custom
   Thunderbird message? Proposal: custom for SDK, bridge adapter for ROS2 standard types.
6. **spconv TensorRT plugin:** CenterPoint requires the `spconv` plugin for TensorRT.
   Evaluate NVIDIA's official `CUDA-BEVFusion` spconv plugin vs. `traveller59/spconv`
   community build for JetPack 6.x stability.
7. **PointPillars drone re-training:** Define anchor sizes, point-cloud range, and class
   taxonomy for the drone domain. Collect or synthesize training data with Thunderbird's
   existing SLAM + LiDAR pipeline.
8. **Auto fallback trigger:** When should `PerceptionEngine` auto-switch from CenterPoint
   to PointPillars on the car platform? Proposal: if rolling-average FPS drops below
   12 Hz for 5 consecutive seconds, switch backend and log a warning.
