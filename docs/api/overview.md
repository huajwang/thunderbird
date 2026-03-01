# API Reference — Overview

> Generated documentation lives at `build/docs/html/index.html` after running
> `cmake --build build --target docs` or `doxygen docs/api/Doxyfile`.

---

## Generating the API Docs

```bash
# Option A: Via CMake (recommended)
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target docs
open build/docs/html/index.html

# Option B: Direct Doxygen invocation
THUNDERBIRD_VERSION=0.2.0 doxygen docs/api/Doxyfile
open build/docs/html/index.html
```

**Requirements:** Doxygen ≥ 1.9.6, Graphviz (for class diagrams).

---

## Header Tiers

The SDK headers are organized into three stability tiers.  Doxygen is
configured to document only Tier 1 and Tier 2 — internal headers are excluded.

### Tier 1 — Public API (stable, SemVer-governed)

These headers form the user-facing contract.  Breaking changes require a major
version bump.

| Header | Purpose |
|--------|---------|
| `thunderbird/device_manager.h` | Primary entry point — lifecycle, callbacks, accessors |
| `thunderbird/types.h` | Core types — `Status`, `DeviceInfo`, `LidarFrame`, `ImuSample`, `CameraFrame` |
| `thunderbird/sensor_data.h` | Data abstraction layer types — `LidarFrame`, `ImuFrame`, `ImageFrame` |
| `thunderbird/time_sync.h` | Time synchronization engine — `SyncedFrame`, `TimeSyncEngine` |
| `thunderbird/diagnostics.h` | Unified metrics — `DiagnosticsManager`, `DiagnosticsSnapshot` |
| `thunderbird/recorder.h` | Session recording — `Recorder`, `RecorderStats` |
| `thunderbird/player.h` | Session playback — `Player`, `PlayerConfig` |
| `thunderbird/version.h` | Compile-time version macros and runtime queries |

### Tier 2 — Extension Points (stable interfaces, may add members)

These headers define abstract interfaces and advanced subsystems.  New virtual
methods or struct members may be added in minor releases.

| Header | Purpose |
|--------|---------|
| `thunderbird/transport.h` | `ITransport` abstract interface |
| `thunderbird/packet_decoder.h` | `IPacketDecoder` abstract interface |
| `thunderbird/perception/perception_engine.h` | 3D perception pipeline |
| `thunderbird/perception/object_detector.h` | Detector backend interface |
| `thunderbird/perception/perception_types.h` | Detection/tracking output types |
| `thunderbird/logging.h` | Per-module logging configuration |
| `thunderbird/clock_service.h` | Clock domain management |
| `thunderbird/device_health_monitor.h` | Health scoring subsystem |
| `thunderbird/lidar_frame_assembler.h` | Multi-packet scan reassembly |

### Tier 3 — Internal (no stability guarantees)

Excluded from Doxygen output.  These may change or be removed without notice.

| Header | Purpose |
|--------|---------|
| `thunderbird/drivers/*` | Simulated + hardware sensor backends |
| `thunderbird/decoders/*` | Vendor-specific packet decoders |
| `thunderbird/protocol.h` | Wire-format definitions |
| `thunderbird/simulated_transport.h` | Test-only loopback transport |

---

## Namespace Layout

```
thunderbird::           ← Core SDK (DeviceManager, types, transports)
  ├── data::            ← Sensor data, time sync, recording
  ├── logging::         ← Per-module spdlog logging
  ├── perception::      ← 3D detection + tracking
  └── odom::            ← SLAM engine, profiler, health
```

All public symbols live inside `inline namespace abi_v0` (managed by
`thunderbird/abi.h`).  This is transparent to user code but enables
link-time ABI version mismatch detection.

---

## Symbol Visibility

| Macro | Meaning |
|-------|---------|
| `THUNDERBIRD_API` | Exported from shared library; part of the public ABI |
| `THUNDERBIRD_LOCAL` | Hidden — never exported |
| `THUNDERBIRD_DEPRECATED(msg)` | Marked as deprecated with migration message |

See `thunderbird/export.h` for platform-specific implementation.

---

## Doxygen Comment Conventions

The project uses `///` (triple-slash) exclusively.  Key rules:

```cpp
/// Brief description (first sentence, auto-extracted by JAVADOC_AUTOBRIEF).
///
/// Extended description with more detail.
///
/// @param config  Full perception configuration.
/// @return true on success, false if initialisation failed.
/// @throws std::bad_alloc on memory exhaustion (rare).
///
/// @note Thread-safe.  May be called from any thread.
/// @see PerceptionConfig for parameter ranges.
bool initialize(const PerceptionConfig& config);
```

| Tag | When to use |
|-----|-------------|
| `@param` | Every function parameter |
| `@return` | Every non-void function |
| `@throws` | If the function can throw |
| `@note` | Thread-safety, performance caveats |
| `@see` | Cross-references |
| `@deprecated` | Use `THUNDERBIRD_DEPRECATED()` macro instead |
| `@example` | Link to a full example program |
