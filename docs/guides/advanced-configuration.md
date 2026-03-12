# Advanced Configuration

All tunable parameters for the Thunderbird SDK — CMake build flags, runtime
configuration structs, logging levels, and perception profiles.

---

## CMake Build Options

See [Building from Source](../getting-started/building-from-source.md) for the
full table.  Key options:

```bash
cmake -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DTHUNDERBIRD_USE_SIMULATED=ON \
    -DTHUNDERBIRD_BUILD_PYTHON=OFF \
    -DTHUNDERBIRD_ENABLE_GPU_PERCEPTION=OFF \
    -DTHUNDERBIRD_HAS_CERES=OFF \
    -DTHUNDERBIRD_LOG_LEVEL_COMPILE=2        # strip trace/debug at compile time
```

---

## DeviceConfig — Master Configuration

All runtime configuration flows through a single `DeviceConfig` struct
passed to `DeviceManager`:

```cpp
thunderbird::DeviceConfig config;

// ── Connection ────────────────────────────────────────────────────
config.uri = "eth://192.168.1.100:7500";  // or "usb://0" or "" (simulated)

// ── Simulated sensor rates (only when THUNDERBIRD_SIMULATED) ─────
config.lidar_hz       = 10.0;
config.imu_hz         = 200.0;
config.camera_fps     = 30.0;
config.camera_width   = 640;
config.camera_height  = 480;

// ── Sync engine (legacy Phase 1) ─────────────────────────────────
config.sync.tolerance_ns   = 50'000'000;   // 50 ms
config.sync.max_bundle_age = 200'000'000;  // 200 ms

// ── Time synchronization (SyncedFrame API) ───────────────────────
config.time_sync.camera_tolerance_ns       = 50'000'000;
config.time_sync.min_imu_samples           = 1;
config.time_sync.enable_drift_estimation   = true;

// ── Connection parameters ────────────────────────────────────────
config.connection.timeout_ms     = 5000;
config.connection.heartbeat_ms   = 1000;
config.connection.client_id      = "my-robot-01";

// ── Retry / reconnect policy ────────────────────────────────────
config.retry.max_retries         = 5;
config.retry.base_delay_ms       = 500;
config.retry.max_delay_ms        = 30000;
config.retry.backoff_multiplier  = 2.0;

// ── Clock service ────────────────────────────────────────────────
config.clock.window_size         = 100;    // OLS sliding window
config.clock.outlier_threshold   = 3.0;    // σ for outlier rejection

// ── Frame assembler ──────────────────────────────────────────────
config.frame_assembler.packets_per_revolution = 76;   // VLP-16
config.frame_assembler.expected_rate_hz       = 10.0;
config.frame_assembler.drop_timeout_ms        = 200;

thunderbird::DeviceManager device(config);
```

---

## Logging Configuration

The SDK uses per-module spdlog loggers that can be tuned independently.

### Modules

| Module | Logger Name | Content |
|--------|-------------|---------|
| `Core` | `tb.core` | DeviceManager, general SDK |
| `Transport` | `tb.transport` | Ethernet, USB, simulated |
| `Parser` | `tb.parser` | PacketParser, PacketDecoder, VLP-16 |
| `TimeSync` | `tb.timesync` | ClockService, TimeSync, SyncEngine |
| `Slam` | `tb.slam` | SlamEngine, SlamHealth, SlamProfiler |
| `Perception` | `tb.perception` | PerceptionEngine, ObjectDetector |
| `Device` | `tb.device` | Device lifecycle, configuration, health |
| `Recorder` | `tb.recorder` | Recorder, Player |
| `Assembler` | `tb.assembler` | FrameAssembler, packet reordering |

### Runtime Level Control

```cpp
#include <thunderbird/logging.h>

// Initialize logging (call once at startup).
thunderbird::logging::LoggingConfig log_cfg;
log_cfg.level         = spdlog::level::info;       // global minimum level
log_cfg.console_enabled = true;
log_cfg.file_enabled  = true;
log_cfg.file_path     = "/var/log/thunderbird.log";
log_cfg.max_file_size = 10 * 1024 * 1024;           // 10 MB per file
log_cfg.max_files     = 3;                           // keep 3 rotated files

thunderbird::logging::init(log_cfg);

// Per-module level override:
thunderbird::logging::set_module_level(
    thunderbird::logging::Module::Transport,
    spdlog::level::trace);
```

### Compile-Time Level Stripping

```bash
# Strip trace + debug calls entirely (zero overhead in release)
cmake -B build -DTHUNDERBIRD_LOG_LEVEL_COMPILE=2   # 0=trace 1=debug 2=info
```

---

## Perception Pipeline Configuration

```cpp
thunderbird::perception::PerceptionConfig perc_config;

// ── Detector backend ─────────────────────────────────────────────
perc_config.detector.backend = thunderbird::perception::DetectorBackend::CpuCluster;
// Options: CpuCluster (default), GpuPointPillars, GpuCenterPoint

// ── Preprocessing ────────────────────────────────────────────────
perc_config.preprocessor.voxel_size       = 0.15;    // metres
perc_config.preprocessor.roi_min          = {-50, -50, -3};
perc_config.preprocessor.roi_max          = { 50,  50,  5};
perc_config.preprocessor.ground_threshold = 0.25;    // metres

// ── Tracker ──────────────────────────────────────────────────────
perc_config.tracker.max_age            = 10;      // frames before deletion
perc_config.tracker.min_hits           = 3;       // frames before confirmed
perc_config.tracker.association_threshold = 3.0;  // Mahalanobis gate

thunderbird::perception::PerceptionEngine engine;
engine.initialize(perc_config);
```

### Profile Presets

| Profile | Use Case | Detector | Motion Model |
|---------|----------|----------|--------------|
| Drone | Aerial platform, CPU-only | `CpuCluster` | `ConstantVelocity` |
| Car | Autonomous driving, GPU available | `GpuPointPillars` | `CTRV` |

---

## Diagnostics Configuration

```cpp
thunderbird::DiagnosticsConfig diag_config;
diag_config.interval       = std::chrono::milliseconds(1000);  // 1 Hz (default)
diag_config.compute_rates  = true;  // derive per-second rates for counters

thunderbird::DeviceConfig config;
// DiagnosticsManager is auto-created inside DeviceManager —
// access via device.diagnostics() after construction.
```

---

## Recording Configuration

```cpp
thunderbird::data::RecorderDeviceInfo dev_info;
dev_info.serial_number    = "TB-2024-0042";
dev_info.firmware_version = "1.2.3";

thunderbird::data::Recorder recorder("session.tbrec", dev_info);
recorder.start();
// ... feed frames ...
recorder.stop();

auto stats = recorder.stats();
std::printf("Recorded: %llu LiDAR, %llu IMU, %llu Camera, %llu bytes\n",
            (unsigned long long)stats.lidar_frames,
            (unsigned long long)stats.imu_frames,
            (unsigned long long)stats.camera_frames,
            (unsigned long long)stats.total_bytes);
```

---

## Calibration Configuration

### CalibrationBundle

Load, modify, and save multi-sensor calibration from YAML:

```cpp
thunderbird::CalibrationBundle bundle;
bundle.load_yaml("calibration.yaml");

bundle.imu_T_lidar.translation = {0.0, 0.0, 0.28};
bundle.refine_imu_T_lidar = true;

bundle.imu_noise.gyro_noise   = 8.0e-4;
bundle.imu_noise.accel_noise  = 1.0e-2;

bundle.save_yaml("calibration_updated.yaml");
```

See [Calibration Guide](calibration.md) for the full YAML schema and
import workflows.

### LiDAR-IMU Calibration (Ceres)

Requires building with `-DTHUNDERBIRD_HAS_CERES=ON`:

| Parameter | Default | Description |
|---|---|---|
| `num_rounds` | 20 | Optimization rounds (voxel grid is rebuilt each round) |
| `voxel_size` | 1.0 | Initial voxel size (metres) |
| `max_octree_depth` | 5 | Maximum octree subdivision |
| `min_points_per_voxel` | 20 | Discard sparse voxels |
| `surface_eigen_ratio` | 16.0 | Eigenvalue ratio for surface features |
| `corner_eigen_ratio` | 9.0 | Eigenvalue ratio for corner features |
| `solver_max_iterations` | 30 | Ceres iterations per round |
| `downsample_size` | 0.2 | Pre-downsample grid (0 = off) |

### LiDAR-Camera Calibration

Dependency-free edge-alignment solver:

| Parameter | Default | Description |
|---|---|---|
| `samples_per_stage` | 5000 | Random samples per refinement stage |
| `num_stages` | 4 | Refinement passes |
| `init_rot_range_deg` | 1.0 | Initial rotation search range (°) |
| `init_trans_range_m` | 0.05 | Initial translation search range (m) |
| `range_decay` | 0.5 | Range reduction factor per stage |
| `edge_threshold` | 30.0 | Pixel gradient magnitude (0–255) |
| `min_depth` | 0.5 | Minimum LiDAR depth (metres) |

### Online Refiner

Runtime ground-plane-based mounting correction:

| Parameter | Default | Description |
|---|---|---|
| `ground_max_height` | -0.5 | Max Z for ground candidates (metres) |
| `min_ground_points` | 100 | Minimum points for a valid ground plane |
| `min_normal_z` | 0.9 | Reject steep planes (0 = vertical, 1 = flat) |
| `min_height` | 1.0 | Minimum estimated mount height (metres) |
| `max_correction_deg` | 2.0 | Warning threshold for rotation (°) |
| `max_correction_height` | 0.10 | Warning threshold for height (metres) |

### SLAM Time-Sync B-Spline

| Parameter | Default | Description |
|---|---|---|
| `use_bspline_interpolation` | `false` | Cubic B-spline instead of linear for scan-boundary IMU |

---

## Environment Variables (planned)

> Note: Environment variable support is **not yet wired into the SDK**. The
> variables below are part of the planned configuration surface and are
> documented here for forward compatibility. In current releases they are
> effectively no-ops — use CMake options and runtime config structs instead.

| Variable | Default | Planned behavior (not yet implemented) |
|----------|---------|-----------------------------------------|
| `THUNDERBIRD_LOG_LEVEL` | `info` | Override the default console log level (e.g. `trace`, `debug`, `info`, `warn`, `error`) |
| `THUNDERBIRD_LOG_FILE` | — | If set, direct logs to the given file path instead of (or in addition to) stderr |
| `THUNDERBIRD_SIMULATED` | — | If set to `1`/`true`, force simulated mode at runtime regardless of build-time defaults |
---

## Further Reading

| Topic | Link |
|-------|------|
| Calibration guide | [Calibration](calibration.md) |
| Time synchronization tuning | [Time Sync Guide](time-synchronization.md) |
| Perception pipeline details | [Perception Design](../design/PERCEPTION_LAYER_DESIGN.md) |
| Hardware setup | [Hardware Guide](../getting-started/hardware-setup.md) |
| All CMake options | [Building from Source](../getting-started/building-from-source.md) |
