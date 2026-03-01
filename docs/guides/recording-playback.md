# Recording & Playback

How to record sensor sessions to `.tbrec` files and replay them
deterministically.

---

## Overview

The Thunderbird SDK includes a **binary recording format** (`.tbrec`) that
captures every LiDAR, IMU, and Camera frame with nanosecond timestamps.
Recordings can be replayed through the same `DeviceManager` API, making
them ideal for:

- Regression testing against known datasets
- Offline perception tuning and evaluation
- Reproducing bugs without hardware

---

## Recording

```cpp
#include <thunderbird/recorder.h>
#include <thunderbird/device_manager.h>

thunderbird::DeviceManager device;
device.connect();
device.start();

// Describe the sensor that produced the data.
thunderbird::data::RecorderDeviceInfo dev_info;
dev_info.serial_number    = "TB-2024-0042";
dev_info.firmware_version = "1.2.3";

thunderbird::data::Recorder recorder("session.tbrec", dev_info);
recorder.start();

// Feed frames (typically from a callback).
device.on_lidar([&](auto frame) {
    recorder.recordLidarFrame(*frame);
});
device.on_imu([&](auto frame) {
    recorder.recordImuFrame(*frame);
});

// ... run for the desired duration ...

recorder.stop();

auto stats = recorder.stats();
std::printf("Recorded: %llu LiDAR, %llu IMU, %llu Camera, %llu bytes\n",
            (unsigned long long)stats.lidar_frames,
            (unsigned long long)stats.imu_frames,
            (unsigned long long)stats.camera_frames,
            (unsigned long long)stats.total_bytes);
```

### Recording Best Practices

- **Disk throughput**: VLP-16 at 10 Hz ≈ 3 MB/s of point data.  Use an SSD
  for sustained recording.
- **Call `stop()`**: The recorder flushes remaining buffers on `stop()`.
  The destructor also calls `stop()` (RAII), but explicit is better.
- **Device info**: Populate `RecorderDeviceInfo` so playback can verify
  format compatibility.

---

## Playback

```cpp
#include <thunderbird/player.h>

thunderbird::data::Player player("session.tbrec");

// Print recording metadata.
auto info = player.info();
std::printf("Duration: %.1f s, Frames: %llu\n",
            info.duration_sec,
            (unsigned long long)info.total_frames);

// Replay at wall-clock speed.
player.play([](const thunderbird::data::LidarFrame& f) {
    process_lidar(f);
}, [](const thunderbird::data::ImuFrame& f) {
    process_imu(f);
});
```

### Playback Modes

| Mode | Method | Behaviour |
|------|--------|-----------|
| Wall-clock | `player.play(...)` | Sleeps between frames to match original timing |
| As-fast-as-possible | `player.play_fast(...)` | No sleep, max throughput |
| Stepped | `player.next()` | Returns one frame per call |

---

## `.tbrec` File Format

```
┌────────────┬──────────────────┬──────────────────┬─────┐
│   Header   │   Frame 0        │   Frame 1        │ ... │
│  (64 B)    │  tag + len + data│  tag + len + data│     │
└────────────┴──────────────────┴──────────────────┴─────┘
```

| Field | Size | Description |
|-------|------|-------------|
| Magic | 4 B | `TBREC` |
| Version | 2 B | Format version (currently 1) |
| Device info | 58 B | Serial, firmware, creation timestamp |
| Frame tag | 1 B | `0x01` LiDAR, `0x02` IMU, `0x03` Camera |
| Payload length | 4 B | Little-endian uint32 |
| Payload | variable | Serialized frame data |

---

## Python API

```python
import spatial_sdk as tb

player = tb.Player("session.tbrec")
for frame in player:
    if frame.type == "lidar":
        pts = frame.points()  # numpy (N, 4) float32
    elif frame.type == "imu":
        acc = frame.accel     # (x, y, z) m/s²
```

---

## Further Reading

- [Recorder Demo](../examples/recorder_demo.cpp) — full example
- [Advanced Configuration](advanced-configuration.md) — recorder config options
- [Troubleshooting](../troubleshooting/troubleshooting.md#recording--playback-issues) — common issues
