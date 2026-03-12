# Time Synchronization

How the Thunderbird SDK aligns data from multiple sensors that run on
independent clocks.

---

## The Problem

A fused LiDAR + IMU + Camera system has **three independent clocks**:

```
Sensor Unit                     Host Computer
┌─────────────────┐            ┌──────────────────┐
│  HW Clock (ns)  │──packets──►│  Host Clock (ns)  │
│  100 MHz TCXO   │            │  steady_clock     │
│                 │            │                   │
│  LiDAR  10 Hz  │            │  Timestamp::now() │
│  IMU   200 Hz  │            │  at packet decode │
│  Camera 30 Hz  │            │                   │
└─────────────────┘            └──────────────────┘
```

**Why this matters:**

- Sensors sample at different rates (10 Hz, 200 Hz, 30 Hz)
- Each sensor stamps data with the device's onboard clock
- The host stamps the arrival time with `steady_clock`
- These two clocks **drift** relative to each other (typically 1–50 ppm)
- Fusing LiDAR + IMU for SLAM or perception requires timestamps in a
  **common reference frame** with sub-millisecond accuracy

---

## Clock Domains

The SDK operates across two clock domains:

| Domain | Source | Resolution | Drift |
|--------|--------|-----------|-------|
| **Hardware** | Sensor TCXO, stamped in packet headers | 1 ns | 0 (reference) |
| **Host** | `std::chrono::steady_clock` at packet decode | ~100 ns | 1–50 ppm vs. hardware |

The `ClockService` (see `clock_service.h`) continuously estimates the
**offset** and **drift rate** between these domains using a sliding-window
Ordinary Least Squares (OLS) regression:

$$\text{host\_ns} = \alpha \cdot \text{hw\_ns} + \beta$$

Where $\alpha$ encodes the drift rate and $\beta$ the current offset.

---

## Synchronization Architecture

```
Device Packets
      │
      ▼
PacketParser ──────────────► hw_timestamp_ns + host_timestamp_ns
      │                                │
      │                    ┌───────────┴───────────┐
      │                    ▼                       ▼
      │            ClockService              TimeSyncEngine
      │         (drift estimation)        (cross-sensor alignment)
      │                    │                       │
      │                    ▼                       ▼
      │          ClockDiagnostics             SyncedFrame
      │         (offset, drift, PPS)     (LiDAR + IMU[] + Camera)
      │                                           │
      └──────────────────────────────────────┐     │
                                              ▼     ▼
                                        DeviceManager
                                     (unified access point)
```

### Three Synchronization Layers

| Layer | Component | Purpose |
|-------|-----------|---------|
| **Clock** | `ClockService` | Estimates offset + drift between HW and Host clocks |
| **Frame** | `TimeSyncEngine` | Aligns LiDAR, IMU, and Camera frames by timestamp |
| **Bundle** | `SyncEngine` (legacy) | Nearest-neighbour LiDAR+IMU+Camera grouping |

---

## TimeSyncEngine — Cross-Sensor Alignment

The `TimeSyncEngine` produces `SyncedFrame` bundles:

```cpp
struct SyncedFrame {
    LidarFrame          lidar;         // anchor frame
    std::vector<ImuFrame> imus;        // all IMUs within the LiDAR period
    std::optional<ImageFrame> camera;  // nearest camera frame (if within tolerance)
    SyncStats           stats;         // sync quality metrics
};
```

**Algorithm:**

1. LiDAR frames arrive at ~10 Hz and act as the **anchor**.
2. For each LiDAR frame, collect all IMU samples whose timestamps fall
   within the current LiDAR period: `[t_prev_lidar, t_current_lidar]`.
3. Find the nearest Camera frame within the configured tolerance
   (default: ±50 ms).
4. Package as a `SyncedFrame` and deliver via Pull or Callback API.

**Timeline:**

```
LiDAR:   |---F1---|---F2---|---F3---|
IMU:     ||||||||||||||||||||||||||||    (200 Hz)
Camera:        C1        C2        C3   (30 Hz)

Result:  [F1 + 20 IMUs + C1]  [F2 + 20 IMUs + C2]  [F3 + ...]
```

### Using the Pull API

```cpp
thunderbird::DeviceManager device;
device.connect();
device.start();

auto& ts = device.time_sync();

// Poll for the latest synced frame.
auto frame = ts.pullSyncedFrame(std::chrono::milliseconds(100));
if (frame) {
    std::printf("LiDAR: %zu pts, IMUs: %zu, Camera: %s\n",
                frame->lidar.points.size(),
                frame->imus.size(),
                frame->camera ? "yes" : "no");
}
```

### Using the Callback API

```cpp
device.on_synced_frame([](const thunderbird::data::SyncedFrame& sf) {
    // Called from the TimeSyncEngine thread — do not block.
    process(sf.lidar, sf.imus, sf.camera);
});
```

---

## ClockService — Drift Estimation

The `ClockService` accumulates `(hw_ns, host_ns)` observation pairs from
every decoded packet and fits a linear model:

```cpp
auto diag = device.clock_service().diagnostics();

std::printf("Offset:     %.1f ns\n",   diag.offset_ns);
std::printf("Drift:      %.3f ns/s\n", diag.drift_ns_per_sec);
std::printf("Uncertainty: %.1f ns\n",   diag.offset_stddev_ns);
std::printf("Calibrated: %s\n",        diag.calibrated ? "yes" : "no");
std::printf("PPS locked: %s\n",        diag.pps_locked ? "yes" : "no");
```

| Field | Meaning |
|-------|---------|
| `offset_ns` | Current host−hardware offset estimate |
| `drift_ns_per_sec` | Drift rate (typical: 1–50 ns/s for TCXO) |
| `offset_stddev_ns` | Estimation uncertainty (σ) |
| `observations` | Total (hw, host) pairs ingested |
| `calibrated` | True once the model has enough samples (default: 20) |
| `pps_locked` | True if a PPS signal is providing ground truth |

### Converting Between Clock Domains

```cpp
auto& clk = device.clock_service();

// Hardware → Host
int64_t host_ns = clk.hardware_to_host(hw_ns);

// Host → Hardware
int64_t hw_ns = clk.host_to_hardware(host_ns);
```

---

## Sync Quality Metrics

Monitor synchronization health via `SyncStats`:

```cpp
auto stats = device.time_sync().stats();
std::printf("Frames produced: %llu\n", (unsigned long long)stats.frames_produced);
std::printf("Camera misses:   %llu\n", (unsigned long long)stats.camera_misses);
std::printf("IMU gaps:        %llu\n", (unsigned long long)stats.imu_gaps);
std::printf("Mean offset:     %.1f ns\n", stats.mean_offset_ns);
std::printf("Drift rate:      %.3f ns/s\n", stats.drift_ns_per_sec);
```

Or use the unified diagnostics system:

```cpp
auto json = device.diagnostics_json();
std::printf("Diagnostics: %s\n", json.c_str());
```

---

## Configuration

```cpp
thunderbird::data::TimeSyncConfig config;
config.camera_tolerance_ns = 50'000'000;  // ±50 ms (default)
config.min_imu_samples     = 1;           // min IMUs per LiDAR frame
config.enable_drift_estimation = true;    // OLS drift tracking

thunderbird::DeviceConfig dev_config;
dev_config.time_sync = config;
thunderbird::DeviceManager device(dev_config);
```

### Clock Service Seeding

If you have a Kalibr time offset, you can **seed** the `ClockService` to
reduce convergence time from ~5 seconds to under 1 second:

```cpp
clock_service.seed_offset(bundle.cameras[0].time_offset_ns);
// Immediately sets calibrated = true with the seeded offset
```

### B-Spline Boundary Interpolation

The SLAM time-sync engine interpolates IMU samples at scan boundaries.
At low IMU rates (< 200 Hz), cubic B-spline interpolation produces
smoother transitions than the default linear interpolation:

```cpp
SlamTimeSyncConfig slam_cfg;
slam_cfg.use_bspline_interpolation = true;  // default: false
```

This uses a uniform cubic B-spline over the IMU block, requiring at
least 4 samples.  At 400+ Hz the difference is negligible.

### PPS Anchoring

For automotive setups with a PPS (pulse-per-second) signal, the
`ClockService` can lock to GPS time:

```cpp
clock_service.observe_pps(host_edge_ns);
// Fires ClockEvent::PpsLocked once stable, ClockEvent::PpsLost on drift
```

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `camera_misses` increasing | Camera FPS too low or timestamp tolerance too tight | Increase `camera_tolerance_ns` |
| `imu_gaps` increasing | IMU samples arriving late or out-of-order | Check USB latency; increase IMU buffer |
| Large `drift_ns_per_sec` | Normal for TCXO oscillators (1–50 ppm) | No action needed — drift is compensated |
| `calibrated = false` after 30s | Too few packets for OLS model | Check that hardware is sending timestamps; or `seed_offset()` from Kalibr |

---

## Further Reading

- [ClockService API](../api/overview.md) — full `ClockService` reference
- [Clock Sync Design Doc](../design/CLOCK_SYNC_DESIGN.md) — internal architecture
- [Calibration Guide](calibration.md) — extrinsic/intrinsic calibration and Kalibr import
- [Diagnostics Guide](diagnostics.md) — unified metrics including clock stats
