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
│                                              └───────────────────┘ │
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
