# Thunderbird SDK — Multi-Sensor Device SDK

A C++20 SDK for interfacing with a fused **LiDAR + IMU + Camera** sensor device.
Includes production-hardened logging, diagnostics, ABI versioning, and
comprehensive documentation.

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                        User Application                             │
│   on_lidar()  on_imu()  on_camera()  on_sync()                     │
└──────────┬──────────┬──────────┬──────────┬────────────────────────┘
           │          │          │          │
┌──────────▼──────────▼──────────▼──────────▼────────────────────────┐
│                      DeviceManager (public API)                     │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐   ┌───────────────────┐ │
│  │ LiDAR    │  │ IMU      │  │ Camera   │   │   SyncEngine      │ │
│  │ Driver   │  │ Driver   │  │ Driver   │──▶│ (nearest-neighbour│ │
│  │ (thread) │  │ (thread) │  │ (thread) │   │  time alignment)  │ │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘   └────────┬──────────┘ │
│       │              │             │                   │            │
│       │              │             │            ┌──────▼──────────┐ │
│       └──────────────┴─────────────┴───────────▶│  ClockService  │ │
│                                                 │ (unified drift │ │
│  ┌─────────────────────────────────────┐        │  compensation) │ │
│  │       ITransport (abstract)         │        └────────────────┘ │
│  │  SimulatedTransport │ USB │ Ethernet│     ┌───────────────────┐ │
│  └─────────────────────────────────────┘     │   SyncBundle      │ │
│                                              │ (LiDAR+IMU+Cam)  │ │
│  ┌──────────────────┐ ┌──────────────────┐   └───────────────────┘ │
│  │ FrameAssembler   │ │ HealthMonitor    │                         │
│  │ (packet → frame) │ │ (temp, V, loss)  │   ┌───────────────────┐ │
│  └──────────────────┘ └──────────────────┘   │ Recorder / Player │ │
│                                              │ (.tbrec binary)   │ │
│                                              └───────────────────┘ │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│                    Calibration Subsystem                             │
│                                                                     │
│  CalibrationBundle ◄── Kalibr YAML import                           │
│   ├─ SensorExtrinsic  (SE(3): inverse, compose)                    │
│   ├─ CameraIntrinsics (per camera, up to 4)                        │
│   └─ ImuNoiseParams   (spectral densities for ESIKF)               │
│                                                                     │
│  ┌────────────────┐ ┌─────────────────┐ ┌────────────────────────┐ │
│  │ RigidTransform │ │ LiDAR-IMU Calib │ │ LiDAR-Camera Calib     │ │
│  │ Horn SVD +     │ │ BALM voxel,     │ │ edge alignment,        │ │
│  │ RANSAC         │ │ optional Ceres  │ │ dependency-free         │ │
│  └────────────────┘ └─────────────────┘ └────────────────────────┘ │
│  ┌────────────────┐ ┌─────────────────┐                            │
│  │ Online Refiner │ │ B-Spline IMU    │                            │
│  │ ground PCA,    │ │ uniform cubic   │                            │
│  │ SLERP accum.   │ │ interpolation   │                            │
│  └────────────────┘ └─────────────────┘                            │
└────────────────────────────┬────────────────────────────────────────┘
                             │ CalibrationBundle
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    AcmeSlamEngine (PImpl)                            │
│                                                                     │
│  feedImu() ──▶ [imu_ring SPSC] ──┐   ┌──────────────────────────┐ │
│  feedCloud()─▶ [cloud_ring SPSC]─┤   │     Worker Thread        │ │
│                                  └──▶│  SlamTimeSync → deskew   │ │
│                                      │  → ESIKF update          │ │
│                                      │  → ikd-Tree insert       │ │
│                                      └───────────┬──────────────┘ │
│                                                   │                │
│  getLatestPose()◄── [pose_ring SPSC] ◄────────────┤                │
│  getLatestOutput()◄─[output_ring SPSC]◄───────────┘                │
└─────────────────────────────────────────────────────────────────────┘
```

### Key design decisions

| Decision | Rationale |
|---|---|
| **`shared_ptr<const T>` for all frames** | Zero-copy fan-out to user callbacks + sync engine; immutable after creation |
| **One thread per sensor driver** | Matches real hardware (independent I/O rates); avoids head-of-line blocking |
| **Lock-free `RingBuffer`** | SPSC pattern for high-throughput paths; drops oldest on overflow |
| **Nearest-neighbour sync** | Simple, deterministic, configurable tolerance — all three sync engines share a unified `ClockService` for drift-compensated timestamps |
| **PImpl in `DeviceManager`** | Stable ABI; hides internal headers from consumers |
| **Dependency-free calibration** | Rigid-transform, LiDAR-camera, and online refiner need no external libraries; LiDAR-IMU uses optional Ceres |
| **`CalibrationBundle` composition** | Single struct aggregates extrinsics, intrinsics, and IMU noise; consumed by SLAM engine, recording, and perception layers |
| **SLAM engine lock-free pipeline** | 4 SPSC ring buffers decouple sensor ingress from the ESIKF worker thread; no mutex on the hot path |

---

## Repository Layout

```
thunderbird/
├── CMakeLists.txt          # Top-level build (version source-of-truth)
├── pyproject.toml          # Python packaging (scikit-build-core)
├── sdk/                    # Core SDK (headers, sources, CMake config)
├── examples/               # C++ example programs
├── eval/                   # Sensitivity & robustness evaluation suite
├── firmware/               # Device firmware stub
├── tests/                  # Unit tests (CTest)
├── python/                 # pybind11 bindings + Python tools & example
├── ros2_bridge/            # ROS 2 node
├── debian/                 # Debian packaging
├── docker/                 # Dockerfiles (runtime + dev)
├── scripts/                # Version extraction, changelog, CI helpers
├── slamd/                  # SLAMD odometry configs (car, drone profiles)
├── .github/workflows/      # CI, Release, Promote, Security workflows
├── ENVIRONMENTS.md         # Environment protection rules
└── SECURITY.md             # Vulnerability reporting policy
```

> **Full tree** with per-file descriptions is in
> [`.github/copilot-instructions.md`](.github/copilot-instructions.md).

---

## Quick Start

### Prerequisites

- **C++20** compiler (GCC 10+, Clang 11+, MSVC 2019+)
- **CMake** ≥ 3.16
- *(Optional)* **pybind11** for Python bindings
- *(Optional)* **ROS 2 Humble/Iron** for the bridge node

### Build (simulated mode — no hardware needed)

```bash
# Clone
git clone <repo-url> thunderbird && cd thunderbird

# Configure + build
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel

# Run tests
cd build && ctest --output-on-failure
```

### Run examples

```bash
./build/examples/basic_streaming
./build/examples/sync_demo
./build/examples/raw_sensors
./build/examples/callback_api_demo
./build/examples/clock_service_demo
./build/examples/comm_layer_demo
./build/examples/frame_assembler_demo
./build/examples/health_monitor_demo
./build/examples/pull_api_demo
./build/examples/recorder_demo
./build/examples/sync_layer_demo
./build/examples/third_party_lidar_demo
./build/examples/import_kalibr_demo

# Firmware demos (require building/running the stub firmware)
./build/examples/firmware_loopback_demo
./build/examples/device_firmware_demo
```

> **Python bindings & ROS 2 bridge** build instructions are in
> [docs/building.md](docs/building.md).

---

## C++ API Reference

### `DeviceManager` — primary entry point

```cpp
#include <thunderbird/thunderbird.h>

thunderbird::DeviceConfig cfg;
cfg.lidar_hz   = 10.0;   // LiDAR scan rate
cfg.imu_hz     = 200.0;  // IMU sample rate
cfg.camera_fps = 30.0;   // Camera frame rate

thunderbird::DeviceManager device(cfg);

// Register callbacks BEFORE start()
device.on_lidar([](std::shared_ptr<const thunderbird::LidarFrame> f) {
    // f->points, f->timestamp, f->sequence_number
});

device.on_imu([](std::shared_ptr<const thunderbird::ImuSample> s) {
    // s->accel, s->gyro, s->temperature, s->timestamp
});

device.on_camera([](std::shared_ptr<const thunderbird::CameraFrame> f) {
    // f->data (raw pixels), f->width, f->height, f->format
});

device.on_sync([](std::shared_ptr<const thunderbird::SyncBundle> b) {
    // b->lidar, b->imu, b->camera — time-aligned
});

device.connect();      // → Status::OK
device.start();        // begins streaming on sensor threads

// ... your processing loop ...

device.stop();
device.disconnect();
```

### Data types

| Type | Fields | Notes |
|---|---|---|
| `Timestamp` | `int64_t nanoseconds` | Steady-clock based; `.to_seconds()` helper |
| `LidarFrame` | `timestamp`, `points[]`, `sequence_number` | Each point: x/y/z/intensity/ring |
| `ImuSample` | `timestamp`, `accel[3]`, `gyro[3]`, `temperature` | SI units (m/s², rad/s, °C) |
| `CameraFrame` | `timestamp`, `width`, `height`, `format`, `data[]` | Raw pixel buffer |
| `SyncBundle` | `reference_time`, `lidar`, `imu`, `camera` | Any member may be `nullptr` |
| `SensorExtrinsic` | `rotation[4]`, `translation[3]` | Quaternion `[w,x,y,z]` + metres; `.inverse()`, `.compose()` |
| `CameraIntrinsics` | `fx`, `fy`, `cx`, `cy`, `width`, `height`, `distortion_model`, `distortion_coeffs[8]` | Pinhole + radtan/equidist/FOV |
| `CalibrationBundle` | `imu_T_lidar`, `cameras[]`, `imu_noise` | YAML load/save; derived `lidar_T_camera()` |

### Sync configuration

```cpp
thunderbird::SyncConfig sync;
sync.tolerance_ns    = 50'000'000;  // 50 ms — max drift between sensors
sync.poll_interval_ms = 5;          // sync thread polling period
sync.reference_sensor = thunderbird::SensorType::LiDAR;

cfg.sync = sync;
```

---

## Calibration Pipeline

Calibration is the foundation of accurate multi-sensor fusion.  The SDK
provides a layered calibration system: a persistent `CalibrationBundle` that
stores all sensor parameters, offline calibration tools, and runtime
refinement that runs inside the SLAM engine.

### CalibrationBundle — loading and saving

All calibration state lives in a single `CalibrationBundle` struct:

```cpp
#include <thunderbird/calibration.h>

thunderbird::CalibrationBundle cal;

// Load from a YAML file
cal.load_yaml("calibration.yaml");

// Or import directly from Kalibr output
thunderbird::calib::importKalibrCamchain("camchain-imucam.yaml", cal);
thunderbird::calib::importKalibrImu("imu.yaml", cal);

// Inspect / modify
cal.imu_T_lidar.translation[2] = 0.05;   // adjust Z offset (metres)
cal.refine_imu_T_lidar = true;            // let SLAM refine this online

// Persist
cal.save_yaml("calibration_updated.yaml");
```

**Calibration YAML schema:**

```yaml
imu_T_lidar:
  rotation: [1.0, 0.0, 0.0, 0.0]       # quaternion [w, x, y, z]
  translation: [0.0, 0.0, 0.0]          # metres [x, y, z]

refine_imu_T_lidar: false                # enable online extrinsic refinement

imu_noise:
  gyro_noise:    1.0e-3    # rad/s/√Hz
  accel_noise:   1.0e-2    # m/s²/√Hz
  gyro_bias_rw:  1.0e-5    # rad/s²/√Hz
  accel_bias_rw: 1.0e-4    # m/s³/√Hz

cameras:
  - label: "cam0"
    intrinsics:
      fx: 500.0
      fy: 500.0
      cx: 320.0
      cy: 240.0
      width: 640
      height: 480
      distortion_model: "radtan"   # radtan | equidistant | fov
      distortion_coeffs: [0.0, 0.0, 0.0, 0.0, 0.0]
    imu_T_camera:
      rotation: [1.0, 0.0, 0.0, 0.0]
      translation: [0.0, 0.0, 0.0]
    time_offset_ns: 0              # from Kalibr temporal calibration
```

### SLAM engine calibration settings

The `SlamEngineConfig` consumes the bundle and adds runtime knobs:

```cpp
thunderbird::odom::SlamEngineConfig slam_cfg;

// ── Embed calibration ───────────────────────────────────────────────
slam_cfg.calibration.load_yaml("calibration.yaml");
slam_cfg.calibration.refine_imu_T_lidar = true;  // online refinement

// ── B-spline IMU interpolation ──────────────────────────────────────
// Uses a uniform cubic B-spline for smooth IMU deskewing instead of
// linear interpolation.  Reduces motion artefacts on aggressive
// platforms (drones, racing cars) at a small CPU cost.
slam_cfg.use_bspline_interpolation = true;

// ── Time-sync tuning ────────────────────────────────────────────────
slam_cfg.enable_drift_compensation = true;
slam_cfg.sort_window_ns = 5'000'000;   // 5 ms reorder window

// ── Deskewing ───────────────────────────────────────────────────────
slam_cfg.deskew.enable = true;
slam_cfg.deskew.imu_integration_substeps = 1;  // 1 = linear, >1 = finer

// ── Sensor rates (for buffer sizing / gap detection) ────────────────
slam_cfg.imu_rate_hz  = 400.0;
slam_cfg.lidar_rate_hz = 10.0;

thunderbird::odom::AcmeSlamEngine engine;
engine.initialize(slam_cfg);
```

**Equivalent YAML** (used by the `slamd` daemon — see `slamd/config/`):

```yaml
slam:
  calibration_file: "calibration.yaml"
  use_bspline_interpolation: false   # true for aggressive platforms
  enable_drift_compensation: true
  sort_window_ns: 5000000
  deskew:
    enable: true
    imu_integration_substeps: 1
```

### When to enable B-spline interpolation

| Platform | `use_bspline_interpolation` | Rationale |
|----------|----------------------------|-----------|
| Car / ground robot | `false` (default) | Smooth motion; linear interpolation is sufficient |
| Drone / handheld | `true` | High angular rates benefit from cubic smoothing |
| Offline post-processing | `true` | No real-time constraint; best quality deskewing |

### Offline calibration tools

The SDK includes three offline calibration algorithms (C++ only):

| Tool | Header | Method | Dependencies |
|------|--------|--------|-------------|
| **Rigid transform** | `calib/rigid_transform.h` | Horn's SVD + RANSAC | None |
| **LiDAR-IMU** | `calib/lidar_imu_calib.h` | BALM voxel optimisation | Optional Ceres (`THUNDERBIRD_HAS_CERES`) |
| **LiDAR-Camera** | `calib/lidar_camera_calib.h` | Multi-stage edge alignment | None |

```cpp
// Example: LiDAR-Camera calibration
#include "calib/lidar_camera_calib.h"

thunderbird::calib::LidarCameraCalibConfig cam_cfg;
cam_cfg.num_stages        = 4;       // refinement stages
cam_cfg.samples_per_stage = 5000;    // random samples per stage
cam_cfg.init_rot_range_deg = 1.0;    // initial search ±1°
cam_cfg.init_trans_range_m = 0.05;   // initial search ±5 cm
cam_cfg.edge_threshold    = 30.0;    // gradient threshold (0–255)

auto result = thunderbird::calib::calibrateLidarCamera(
    points, n_points, edge_image, intrinsics, initial_extrinsic, cam_cfg);

if (result.converged) {
    bundle.cameras[0].imu_T_camera = /* compose with result */;
    bundle.save_yaml("calibration_refined.yaml");
}
```

### Online extrinsic refinement

When `refine_imu_T_lidar = true`, the SLAM engine runs a per-frame
ground-plane PCA to continuously correct pitch, roll, and height:

```cpp
// Enable online refinement
slam_cfg.calibration.refine_imu_T_lidar = true;

// The OnlineRefinerConfig controls safety bounds (internal defaults):
//   ground_max_height    = -0.5 m    — Z cutoff for ground candidates
//   min_ground_points    = 100       — minimum points for valid fit
//   max_correction_deg   = 2.0°      — safety clamp
//   max_correction_height = 0.10 m   — safety clamp
```

### Kalibr import

The `import_kalibr_demo` example imports Kalibr output in a single command:

```bash
./build/examples/import_kalibr_demo \
  --camchain camchain-imucam.yaml \
  --imu imu.yaml \
  -o calibration.yaml
```

This populates `cameras[]` (intrinsics, extrinsics, temporal offsets) and
`imu_noise` from Kalibr's YAML format, then saves a Thunderbird-native
calibration file.

---

## Time Synchronization — How It Works

> See [docs/time-synchronization.md](docs/time-synchronization.md) for the
> timing diagram and per-cycle algorithm details.

---

## CI/CD & Security

> See [docs/ci-cd.md](docs/ci-cd.md) for workflows, release artifacts,
> security tooling, and the release process.
>
> See [SECURITY.md](SECURITY.md) for vulnerability reporting.
> See [ENVIRONMENTS.md](ENVIRONMENTS.md) for environment protection rules.

---

### Packaging — Docker, Debian & Python Wheels

> Docker images, Debian packages, and Python wheel build/install instructions
> are in [docs/packaging.md](docs/packaging.md).

---

## Versioning Strategy

**Single source of truth: Git tag** (`vX.Y.Z`). The version propagates automatically
to C++ headers, Python wheels, Debian packages, and Docker images via CI.

### Checking the version at compile time

```cpp
#include <thunderbird/version.h>

static_assert(THUNDERBIRD_VERSION_AT_LEAST(1, 0, 0), "Need SDK ≥ 1.0.0");
printf("SDK %s\n", thunderbird::version());  // "1.2.3"
```

### Checking the version in Python

```python
import spatial_sdk
print(spatial_sdk.__version__)  # "1.2.3"
```

> **Version propagation internals** (extract-version.sh, CMake override,
> Python resolution chain, Debian substitution, Docker build-arg) are
> documented in [`.github/copilot-instructions.md`](.github/copilot-instructions.md).

---

## Implementation Phases

### Phase 1 — Core infrastructure ✅
- Data types with dual timestamps (hardware + host)
- `ITransport` abstraction and `SimulatedTransport`
- Per-sensor driver interface + simulated backends
- Lock-free `RingBuffer` for high-throughput paths
- `SyncEngine` with nearest-neighbour alignment
- `DeviceManager` with PImpl pattern
- C++ examples and unit tests
- ROS 2 bridge node (PointCloud2, Imu, Image publishers)
- Python bindings via pybind11

### Phase 2 — Hardware integration ✅
- Real USB/Ethernet transport drivers (`SO_TIMESTAMPING`)
- Vendor-specific packet parsing (`IPacketDecoder`, `DecoderFactory`, VLP-16)
- Hardware PTP/PPS timestamp recovery (`ClockService`)
- Clock-offset estimation (linear regression on HW↔host timestamps)
- `LidarFrameAssembler` (packet → frame reconstruction)
- `DeviceHealthMonitor` (temperature, voltage, packet-loss tracking)
- Reconnection / watchdog logic
- Recording / playback (`Recorder`, `Player`, `.tbrec` binary format)

### Phase 3 — Production hardening ✅
- Configurable logging framework (spdlog, per-module levels, JSON sink)
- Thread-safe `DiagnosticsManager` with snapshot API
- API versioning & ABI stability (`abi_v0` namespace, `THUNDERBIRD_API` exports)
- Comprehensive documentation (16 files: guides, troubleshooting, API reference)

### Phase 4 — Calibration subsystem ✅
- `CalibrationBundle` with YAML serialization and Kalibr import
- `SensorExtrinsic` SE(3) transforms with inverse/compose
- Rigid transform solver (Horn's SVD + RANSAC)
- LiDAR-IMU calibration (BALM, optional Ceres)
- LiDAR-Camera calibration (edge alignment, dependency-free)
- Online extrinsic refiner (ground-plane PCA)
- Uniform cubic B-spline IMU interpolation
- `.tbrec` → ROS 2 bag and `.tbrec` → PCD/PNG converter tools
- SLAMD odometry profiles (car, drone)

### Phase 5 — Perception-ready extensions (next)
- GPU-accelerated point cloud preprocessing
- Multi-device support (multiple `DeviceManager` instances)
- Object detection pipeline

---

## License

MIT — see `LICENSE` for details.
