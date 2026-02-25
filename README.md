# Thunderbird SDK — Multi-Sensor Device SDK (PoC)

A C++17 SDK for interfacing with a fused **LiDAR + IMU + Camera** sensor device.
This PoC focuses on **SDK infrastructure, data flow, and time synchronization** —
no AI/perception algorithms are included yet.

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
│  └────┬─────┘  └────┬─────┘  └────┬─────┘   └─────────┬─────────┘ │
│       │              │             │                    │           │
│  ┌────▼──────────────▼─────────────▼───┐     ┌─────────▼─────────┐ │
│  │       ITransport (abstract)         │     │   SyncBundle      │ │
│  │  SimulatedTransport │ USB │ Ethernet│     │ (LiDAR+IMU+Cam)  │ │
│  └─────────────────────────────────────┘     └───────────────────┘ │
└─────────────────────────────────────────────────────────────────────┘
```

### Key design decisions

| Decision | Rationale |
|---|---|
| **`shared_ptr<const T>` for all frames** | Zero-copy fan-out to user callbacks + sync engine; immutable after creation |
| **One thread per sensor driver** | Matches real hardware (independent I/O rates); avoids head-of-line blocking |
| **Lock-free `RingBuffer`** | SPSC pattern for high-throughput paths; drops oldest on overflow |
| **Nearest-neighbour sync** | Simple, deterministic, configurable tolerance — good enough for PoC |
| **PImpl in `DeviceManager`** | Stable ABI; hides internal headers from consumers |

---

## Repository Layout

```
thunderbird/
├── CMakeLists.txt              # Top-level build
├── sdk/
│   ├── CMakeLists.txt
│   ├── include/thunderbird/
│   │   ├── thunderbird.h          # Umbrella header
│   │   ├── types.h                # Timestamp, data structs, enums
│   │   ├── transport.h            # ITransport interface
│   │   ├── simulated_transport.h  # Loopback transport
│   │   ├── sensor_driver.h        # ISensorDriver interface
│   │   ├── ring_buffer.h          # Lock-free SPSC ring buffer
│   │   ├── sync_engine.h          # Time-alignment engine
│   │   ├── device_manager.h       # Main public API
│   │   └── drivers/
│   │       ├── simulated_lidar.h
│   │       ├── simulated_imu.h
│   │       └── simulated_camera.h
│   └── src/
│       └── device_manager.cpp
├── examples/
│   ├── basic_streaming.cpp        # Connect + stream + print stats
│   ├── sync_demo.cpp              # Time-aligned bundle demo
│   └── raw_sensors.cpp            # Deep-dive into raw sensor data
├── tests/
│   ├── test_ring_buffer.cpp
│   ├── test_sync_engine.cpp
│   └── test_device_manager.cpp
├── python/
│   ├── CMakeLists.txt
│   ├── src/bindings.cpp           # pybind11 module
│   └── example.py                 # Python usage demo
└── ros2_bridge/
    ├── CMakeLists.txt
    ├── package.xml
    └── src/thunderbird_ros2_node.cpp
```

---

## Quick Start

### Prerequisites

- **C++17** compiler (GCC 9+, Clang 10+, MSVC 2019+)
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
```

### Build with Python bindings

```bash
pip install pybind11
cmake -B build -DTHUNDERBIRD_BUILD_PYTHON=ON
cmake --build build --parallel

# Run
cd python
PYTHONPATH=../build/python python example.py
```

### Build the ROS 2 bridge

```bash
# Inside a ROS 2 workspace
ln -s /path/to/thunderbird/ros2_bridge src/thunderbird_ros2_bridge
colcon build --packages-select thunderbird_ros2_bridge
source install/setup.bash
ros2 run thunderbird_ros2_bridge thunderbird_ros2_node
```

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

```
Time ──────────────────────────────────────────────────────▶

LiDAR  ─── L1 ─────────────── L2 ─────────────── L3 ───
            10 Hz

IMU    ─ I1 I2 I3 I4 I5 I6 I7 I8 I9 I10 I11 I12 I13 ──
            200 Hz

Camera ──── C1 ─────── C2 ─────── C3 ─────── C4 ───────
            30 Hz

SyncEngine takes each LiDAR frame as reference and finds the
nearest IMU sample and Camera frame within ±tolerance_ns.

Bundle 1:  L1 + I_nearest + C_nearest
Bundle 2:  L2 + I_nearest + C_nearest
...
```

**Algorithm** (per sync cycle):
1. Pop the oldest unconsumed LiDAR frame → `ref_ts`
2. Search IMU queue for sample closest to `ref_ts` within tolerance
3. Search Camera queue for frame closest to `ref_ts` within tolerance
4. If Camera is available, emit `SyncBundle`; evict consumed data
5. If not, wait until next poll cycle

---

## Implementation Plan (step-by-step)

### Phase 1 — Core infrastructure (this PoC) ✅
1. Define data types with dual timestamps (hardware + host)
2. Implement `ITransport` and `SimulatedTransport`
3. Build per-sensor driver interface + simulated backends
4. Implement lock-free `RingBuffer` for high-throughput paths
5. Build `SyncEngine` with nearest-neighbour alignment
6. Wire everything into `DeviceManager` with PImpl pattern
7. Add C++ examples and unit tests
8. Create ROS 2 bridge node (PointCloud2, Imu, Image publishers)
9. Add Python bindings via pybind11

### Phase 2 — Hardware integration (next)
- [ ] Implement real USB/Ethernet transport drivers
- [ ] Parse vendor-specific packet formats (LiDAR, IMU, Camera)
- [ ] Hardware PTP/PPS timestamp recovery
- [ ] Clock-offset estimation (linear regression on HW↔host timestamps)
- [ ] Reconnection / watchdog logic

### Phase 3 — Production hardening
- [ ] Configurable logging framework (spdlog)
- [ ] Thread-safe statistics / diagnostics endpoint
- [ ] CI pipeline (CMake presets, cross-compilation, sanitizers)
- [ ] API versioning & ABI stability guarantees
- [ ] Documentation generation (Doxygen)

### Phase 4 — Perception-ready extensions
- [ ] GPU-accelerated point cloud preprocessing
- [ ] Camera intrinsic / extrinsic calibration storage
- [ ] Multi-device support (multiple `DeviceManager` instances)
- [ ] Recording / playback (rosbag2 or custom binary format)

---

## License

MIT — see `LICENSE` for details.
