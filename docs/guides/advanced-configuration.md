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
| `Recorder` | `tb.recorder` | Recorder, Player |

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

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `THUNDERBIRD_LOG_LEVEL` | `info` | Console log level override |
| `THUNDERBIRD_LOG_FILE` | — | File path for log output |
| `THUNDERBIRD_SIMULATED` | — | Force simulated mode at runtime |

---

## Further Reading

| Topic | Link |
|-------|------|
| Time synchronization tuning | [Time Sync Guide](time-synchronization.md) |
| Perception pipeline details | [Perception Design](../design/PERCEPTION_LAYER_DESIGN.md) |
| Hardware setup | [Hardware Guide](../getting-started/hardware-setup.md) |
| All CMake options | [Building from Source](../getting-started/building-from-source.md) |
