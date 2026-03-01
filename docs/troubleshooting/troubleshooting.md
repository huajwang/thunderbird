# Troubleshooting

Common problems, their causes, and fixes.

---

## Connection Issues

| Symptom | Cause | Fix |
|---------|-------|-----|
| `connect()` throws `connection_timeout` | Sensor not on same subnet | Set host NIC to `192.168.1.x`, verify with `ping 192.168.1.100` |
| `connect()` throws `connection_timeout` (USB) | Missing udev permissions | Install the [udev rule](../getting-started/hardware-setup.md#udev-rules) and replug |
| Connection drops every few minutes | Heartbeat timeout too short | Increase `config.connection.heartbeat_ms` (default: 1000) |
| `reconnect_count` increasing | Ethernet cable/switch issue | Check physical connection; try a direct cable |
| Flap detection triggers (`flap_detected = true`) | Rapid connect/disconnect cycle | Check power supply stability; inspect USB hub |

---

## Build Issues

| Symptom | Cause | Fix |
|---------|-------|-----|
| `CMake Error: Could not find pybind11` | Python bindings enabled but pybind11 not found | Install `pybind11-dev` or set `-DTHUNDERBIRD_BUILD_PYTHON=OFF` |
| `fatal error: spdlog/spdlog.h: No such file or directory` | spdlog not installed | `sudo apt install libspdlog-dev` (≥ 1.10) |
| `undefined reference to thunderbird::*` | Linking order wrong | Link with `-lthunderbird-sdk` **after** your object files |
| `ABI mismatch: built v1, loaded v2` | SDK library/header version mismatch | Rebuild your app against the same SDK version |
| Build fails on macOS with Xcode ≥ 15 | Linker warnings treated as errors | Add `-DCMAKE_EXE_LINKER_FLAGS="-Wl,-no_warn_duplicate_libraries"` |

---

## Data Issues

| Symptom | Cause | Fix |
|---------|-------|-----|
| LiDAR frames contain 0 points | Simulated mode with sensor URI set | Clear `config.uri` or set `-DTHUNDERBIRD_USE_SIMULATED=ON` |
| Points all at (0, 0, 0) | Firmware not returning distance data | Update sensor firmware; check return mode |
| `partial_frames` increasing | Packets lost on the network | Increase socket buffer: `sudo sysctl -w net.core.rmem_max=26214400` |
| `dropped_packets_total` rising | Host CPU cannot keep up | Reduce LiDAR return mode; check for CPU throttling |
| IMU values look wrong (scale) | Units mismatch (rad/s vs. deg/s) | SDK outputs rad/s for gyro, m/s² for accel — check your consumer |

---

## Time Synchronization Issues

| Symptom | Cause | Fix |
|---------|-------|-----|
| `camera_misses` increasing | Camera tolerance too tight or FPS low | Increase `config.time_sync.camera_tolerance_ns` |
| `imu_gaps` increasing | IMU samples arriving late | Check USB latency; try lower IMU rate |
| `calibrated = false` after 30+ seconds | Too few timestamped packets | Verify hardware is emitting timestamp headers |
| Large negative `offset_ns` | Host clock jumped (NTP step) | Use `steady_clock` (default); avoid system clock for timing |
| `drift_ns_per_sec` > 100 | Faulty oscillator or thermal stress | Normal range is 1–50 ns/s; check sensor operating temperature |

See [Time Synchronization Guide](../guides/time-synchronization.md) for
detailed architecture.

---

## Perception Issues

| Symptom | Cause | Fix |
|---------|-------|-----|
| No detections from `PerceptionEngine` | ROI too small / voxel size too large | Check `preprocessor.roi_min`/`roi_max` and `voxel_size` |
| Ghost objects appearing | Ground points not filtered | Decrease `preprocessor.ground_threshold` |
| Tracker IDs cycling rapidly | `max_age` too low or association gate too tight | Increase `tracker.max_age`; relax `association_threshold` |
| GPU backend segfaults | CUDA/driver version mismatch | Verify `nvidia-smi` works; rebuild with matching CUDA toolkit |

---

## Recording & Playback Issues

| Symptom | Cause | Fix |
|---------|-------|-----|
| `.tbrec` file is 0 bytes | `recorder.stop()` never called | Ensure `stop()` is called before destruction (RAII handles this) |
| Playback speed wrong | Clock drift not recorded | Re-record with drift estimation enabled |
| `Player` skips frames | File corrupted or truncated | Check disk space during recording; verify CRC |

---

## Logging

Enable verbose logging to diagnose SDK-internal issues:

```cpp
#include <thunderbird/logging.h>

// Console: show everything.
thunderbird::logging::LoggingConfig log_cfg{};
log_cfg.console_enabled = true;
log_cfg.level = spdlog::level::trace;
log_cfg.file_enabled = false;
thunderbird::logging::init(log_cfg);

// Or target a single module:
thunderbird::logging::set_module_level(
    thunderbird::logging::Module::Transport,
    spdlog::level::trace);
```

---

## Getting Help

1. **Diagnostics JSON** — call `device.diagnostics_json()` and include the
   output in any bug report.
2. **Log file** — reproduce with `file_level = trace` and attach the log.
3. **Recording** — capture a `.tbrec` session so maintainers can replay.
4. **GitHub Issues** — file at
   `https://github.com/huajwang/thunderbird/issues` with logs, config,
   and diagnostics JSON.
