# Perception Pipeline Guide

How the Thunderbird SDK converts raw LiDAR point clouds into tracked 3-D
object detections.

---

## Pipeline Overview

```
LidarFrame (N points)
      │
      ▼
┌──────────────┐    ┌──────────────┐    ┌──────────────────┐
│ Preprocessor │───►│   Detector   │───►│ Multi-Object     │
│              │    │              │    │ Tracker (MOT)    │
│ • Voxel grid │    │ CPU: cluster │    │ • Kalman filter  │
│ • ROI crop   │    │ GPU: PointP. │    │ • Hungarian      │
│ • Ground rm  │    │ GPU: CenterP │    │ • Track mgmt    │
└──────────────┘    └──────────────┘    └──────────────────┘
                                                │
                                                ▼
                                        TrackedObject[]
                                    (id, class, bbox, vel)
```

---

## Stages

### 1. Preprocessor

Reduces the raw point cloud to the region of interest and a manageable
density.

| Step | Purpose | Key Parameter |
|------|---------|---------------|
| ROI crop | Discard points outside the volume | `roi_min`, `roi_max` |
| Ground removal | Remove road / floor surface | `ground_threshold` (m) |
| Voxel down-sample | Reduce density for detector | `voxel_size` (m) |

```cpp
perc_config.preprocessor.voxel_size       = 0.15;   // 15 cm voxels
perc_config.preprocessor.roi_min          = {-50, -50, -3};
perc_config.preprocessor.roi_max          = { 50,  50,  5};
perc_config.preprocessor.ground_threshold = 0.25;
```

### 2. Detector

Identifies object candidates in a single frame (no temporal context).

| Backend | Requires | Latency | Accuracy |
|---------|----------|---------|----------|
| `CpuCluster` | CPU only | ~8 ms | Good for sparse scenes |
| `GpuPointPillars` | CUDA ≥ 11 | ~4 ms | High |
| `GpuCenterPoint` | CUDA ≥ 11 | ~6 ms | Highest |

Selection:

```cpp
perc_config.detector.backend =
    thunderbird::perception::DetectorBackend::CpuCluster;
```

### 3. Multi-Object Tracker (MOT)

Associates detections across frames and estimates velocity.

**Kalman Filter** — Constant-velocity (CV) or Constant Turn Rate and
Velocity (CTRV) motion model per tracked object.

**Hungarian Algorithm** — Optimal one-to-one assignment of new detections
to existing tracks, minimising Mahalanobis distance.

**Track Lifecycle:**

```
                 hits ≥ min_hits
  Tentative ─────────────────────► Confirmed ──► Lost ──► Deleted
      ▲                                           │ age > max_age
      │ new detection (unmatched)                  └──────────────►
```

```cpp
perc_config.tracker.max_age               = 10;   // frames before delete
perc_config.tracker.min_hits              = 3;    // frames before confirm
perc_config.tracker.association_threshold = 3.0;  // Mahalanobis gate (σ)
```

---

## Using the PerceptionEngine

```cpp
#include <thunderbird/perception_engine.h>
#include <thunderbird/device_manager.h>

thunderbird::perception::PerceptionConfig perc_config;
// ... configure as above ...

thunderbird::perception::PerceptionEngine perception;
perception.initialize(perc_config);

thunderbird::DeviceManager device;
device.connect();
device.start();

device.on_lidar([&](const thunderbird::data::LidarFrame& f) {
    auto objects = perception.process(f);
    for (const auto& obj : objects) {
        std::printf("ID %u  class=%d  pos=(%.1f, %.1f, %.1f)  vel=(%.1f, %.1f)\n",
                    obj.id, static_cast<int>(obj.classification),
                    obj.position.x, obj.position.y, obj.position.z,
                    obj.velocity.x, obj.velocity.y);
    }
});
```

---

## TrackedObject Output

```cpp
struct TrackedObject {
    uint32_t id;                  // unique track ID
    ObjectClass classification;   // Car, Pedestrian, Cyclist, Unknown
    Point3d position;             // centre (metres)
    Point3d dimensions;           // length, width, height
    float heading;                // yaw (radians)
    Point2d velocity;             // (vx, vy) m/s
    uint32_t age;                 // frames since first detection
    float confidence;             // 0.0–1.0
};
```

---

## Evaluation & Benchmarks

The SDK ships with an evaluation harness under `eval/`:

```bash
cd build
cmake --build . --target eval_runner

# Run with the KITTI-format ground truth.
./eval/eval_runner --gt kitti_labels/ --dt detections/ --metric MOTA
```

Metrics produced:

| Metric | Description |
|--------|-------------|
| MOTA | Multi-Object Tracking Accuracy |
| MOTP | Multi-Object Tracking Precision |
| IDS | Identity switches |
| FP / FN | False positives / negatives |
| Hz | Pipeline throughput |

See [Baseline Results](../eval/BASELINE_RESULTS.md) for reference numbers.

---

## Further Reading

- [Perception Layer Design](../design/PERCEPTION_LAYER_DESIGN.md) — internal architecture
- [Advanced Configuration](advanced-configuration.md) — all perception tuning knobs
- [Sensitivity Report](../eval/SENSITIVITY_REPORT.md) — parameter sweep results
- [Robustness Report](../eval/ROBUSTNESS_REPORT.md) — edge-case analysis
