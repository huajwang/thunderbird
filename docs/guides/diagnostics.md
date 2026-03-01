# Diagnostics Guide

How to use the unified diagnostics system to monitor SDK health and
performance at runtime.

---

## Overview

The `DiagnosticsManager` collects metrics from all SDK subsystems on a
background thread at a configurable interval (default: 1 Hz).  Each metric
is tagged as a **Counter** (monotonic), **Gauge** (instantaneous), or
**Flag** (boolean).

```
┌─────────────┐  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐
│  TimeSyncEng │  │ ClockService │  │ Assembler    │  │ HealthMonitor│
│  .stats()   │  │ .diagnostics │  │ .stats()     │  │ .snapshot()  │
└──────┬──────┘  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘
       │                │                  │                  │
       └────────┬───────┴────────┬─────────┘                  │
                │  collectors    │                             │
                ▼                ▼                             ▼
         ┌─────────────────────────────────────────────────────┐
         │              DiagnosticsManager                     │
         │         (background thread, 1 Hz)                   │
         └───────────────────────┬─────────────────────────────┘
                                 │
                    ┌────────────┴────────────┐
                    ▼                         ▼
           latest_snapshot()              to_json()
```

---

## Quick Start

```cpp
thunderbird::DeviceManager device;
device.connect();
device.start();

// Wait for data to accumulate.
std::this_thread::sleep_for(std::chrono::seconds(2));

// Get metrics as a snapshot or JSON.
auto snap = device.diagnostics_snapshot();
auto json = device.diagnostics_json();

std::printf("Diagnostics:\n%s\n", json.c_str());
```

---

## Available Metrics

### `time_sync` Group

| Metric | Type | Description |
|--------|------|-------------|
| `frames_produced` | Counter | Total SyncedFrames emitted |
| `camera_misses` | Counter | LiDAR frames with no camera match |
| `imu_gaps` | Counter | LiDAR frames with empty IMU block |
| `mean_offset_ns` | Gauge | Running mean lidar↔camera offset |
| `drift_ns_per_sec` | Gauge | Latest drift estimate |

### `clock` Group

| Metric | Type | Description |
|--------|------|-------------|
| `offset_ns` | Gauge | Current host−hardware offset |
| `drift_ns_per_sec` | Gauge | Clock drift rate |
| `offset_stddev_ns` | Gauge | Offset uncertainty (σ) |
| `observations` | Counter | Total (hw, host) pairs ingested |
| `jumps_detected` | Counter | Time jumps detected |
| `outliers_rejected` | Counter | Samples rejected by filter |
| `calibrated` | Flag | Clock model calibrated |
| `pps_locked` | Flag | PPS signal active |

### `assembler` Group

| Metric | Type | Description |
|--------|------|-------------|
| `frames_emitted` | Counter | Complete LiDAR frames produced |
| `partial_frames` | Counter | Incomplete frames seen |
| `packets_ingested` | Counter | Total packets fed |
| `points_ingested` | Counter | Total points fed |
| `dropped_packets_total` | Counter | Packets lost |
| `avg_points_per_frame` | Gauge | Rolling average points/frame |
| `measured_rate_hz` | Gauge | Measured output frame rate |

### `device_health` Group (hardware mode only)

| Metric | Type | Description |
|--------|------|-------------|
| `health_score` | Gauge | Aggregate health (0.0–1.0) |
| `lidar_hz` | Gauge | Measured LiDAR rate |
| `imu_hz` | Gauge | Measured IMU rate |
| `camera_fps` | Gauge | Measured camera FPS |
| `bytes_total` | Counter | Total bytes received |
| `packets_total` | Counter | Total packets received |
| `crc_errors_total` | Counter | CRC integrity failures |
| `reconnect_count` | Counter | Connection re-establishments |
| `heartbeat_rtt_ms` | Gauge | Heartbeat round-trip time |
| `flap_detected` | Flag | Connection flapping |

---

## Rate Computation

When `DiagnosticsConfig::compute_rates` is enabled (default), Counter metrics
are automatically converted to per-second rates with a `.total` suffix
preserving the raw value:

```json
{
  "assembler": {
    "frames_emitted": 10.02,
    "frames_emitted.total": 1203,
    "packets_ingested": 760.5,
    "packets_ingested.total": 91260
  }
}
```

---

## Custom Collectors

Register your own metrics collector:

```cpp
auto& diag = device.diagnostics();

diag.register_collector("my_app", []() -> thunderbird::MetricMap {
    return {
        {"processed_frames", {thunderbird::MetricType::Counter, app_frame_count}},
        {"latency_ms",       {thunderbird::MetricType::Gauge,   app_latency}},
        {"overloaded",       {thunderbird::MetricType::Flag,    is_overloaded ? 1.0 : 0.0}},
    };
});
```

Collectors must be registered **before** `device.start()`.

---

## JSON Output Format

```json
{
  "time_sync": {
    "frames_produced": 42,
    "camera_misses": 0,
    "imu_gaps": 1,
    "mean_offset_ns": 1234567.0,
    "drift_ns_per_sec": 12.3
  },
  "clock": {
    "offset_ns": 98765.0,
    "drift_ns_per_sec": 8.2,
    "calibrated": true,
    "pps_locked": false
  },
  "assembler": { ... },
  "device_health": { ... }
}
```
