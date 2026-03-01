# Thunderbird SDK — LiDAR Frame Assembler Design

> **Status:** Design proposal  
> **Author:** Perception Systems Architecture  
> **Date:** 2026-03-01  
> **Depends on:** `PacketParser`, `IPacketDecoder`, `protocol.h`, `types.h`, `slam_types.h`

---

## 0. Existing Code Audit

### 0.1 What exists: the complete LiDAR data path

```
Device  ──UDP/TCP──►  PacketParser::dispatch_lidar()
                              │
                      LidarSubHeader {
                          num_points,
                          azimuth_start,        ◄── present but UNUSED
                          azimuth_end           ◄── present but UNUSED
                      }
                      + LidarWirePoint[num_points]
                              │
                      ┌───────▼─────────┐
                      │  LidarFrame     │    ◄── ONE per PACKET, not per scan
                      │  (types.h)      │
                      │  .points        │    ~50–200 points/packet
                      │  .sequence_number│
                      │  .timestamp     │
                      │  .host_timestamp│
                      └───────┬─────────┘
                              │
                 ┌────────────┼────────────────────────────┐
                 ▼            ▼                             ▼
           SyncEngine    DeviceManager::lidar_cb      SlamDaemon::on_lidar
           (types.h)     ↓ convert                    ↓ convert
                    data::LidarFrame              odom::PointCloudFrame
                    ↓ feed                        ↓ feedPointCloud
                    TimeSyncEngine                AcmeSlamEngine
```

**Critical finding:** every downstream consumer receives **per-packet** data
containing 50–200 points, not a complete 360° sweep of ~28,000 points.  The
`SlamDaemon` callback naively wraps each per-packet `LidarFrame` into an
`odom::PointCloudFrame` and feeds it to the ESIKF as if it were a full scan.

### 0.2 Inventory of existing structures

| Structure | File | Points per instance | Full scan? | Azimuth info? |
|-----------|------|-------------------|------------|---------------|
| `LidarFrame` | `types.h` | Variable (per packet) | **No** | No — has `sequence_number` but no azimuth |
| `LidarSubHeader` | `protocol.h` | N/A (wire sub-header) | No (segment) | **Yes** — `azimuth_start`, `azimuth_end` (degrees) |
| `data::LidarFrame` | `sensor_data.h` | Variable | **No** | No |
| `data::PointXYZIT` | `sensor_data.h` | 1 | — | Has `timestamp_offset_ns` (designed for motion compensation, set to 0 in practice) |
| `odom::PointCloudFrame` | `slam_types.h` | Variable | **Intended** to be one sweep | No azimuth tracking |
| `odom::PointXYZIT` | `slam_types.h` | 1 | — | Has `dt_ns` (designed for deskewing, populated only in test/eval code) |
| `LidarPoint` | `types.h` | 1 | — | Has `ring` but no azimuth |

### 0.3 What the simulated path does (and why it works)

`SimulatedLidarDriver` generates a **full 360° scan** (16 rings × 360
columns = 5,760 points) in a single `LidarFrame` callback at 10 Hz.  This
is why tests and benchmarks produce reasonable results — they receive actual
full-sweep point clouds.

Real hardware (Thunderbird native or VLP-16) sends 10–75 packets per
revolution.  **Nobody assembles them.**

### 0.4 VLP-16 decoder: close but no cigar

`VelodyneVlp16Decoder` (in `VENDOR_PARSER_DESIGN.md`) has tantalizing
member variables:

```cpp
float frame_azimuth_start_{0.0f};     // ◄── stored, never used
float frame_azimuth_end_{0.0f};       // ◄── stored, never used
uint32_t frame_seq_{0};
```

The decoder fires `lidar_cb_` once per UDP packet (~192 points).  It tracks
azimuth ranges per-packet but never accumulates packets into a full sweep.

### 0.5 Protocol.h: azimuth is on the wire

```cpp
struct LidarSubHeader {
    uint32_t num_points{0};
    float    azimuth_start{0};   // degrees — start of this scan segment
    float    azimuth_end{0};     // degrees — end of this scan segment
};
```

Both the Thunderbird native protocol and the VLP-16 decoder know the
azimuth range of each packet.  The information to detect revolution
boundaries exists — it's just never consumed.

### 0.6 Gap analysis summary

| Requirement | Status | Gap |
|-------------|--------|-----|
| Assemble packets into 360° frame | **NOT DONE** | Core missing component — nobody accumulates per-packet data into full sweeps |
| Handle packet drop | **NOT DONE** | No azimuth gap detection; `SequenceTracker` exists in the vendor parser design but isn't wired to frame assembly |
| Configurable frame rate | **PARTIAL** — `SimulatedLidarDriver` has `scan_rate_hz` | Real hardware has no frame-rate concept (it's determined by sensor rotation speed); assembler must detect it, not configure it |
| Support partial-frame timeout | **NOT DONE** | No timeout for incomplete sweeps |
| Point cloud in device coordinate frame | **DONE** — `LidarWirePoint` and `LidarPoint` are already (x, y, z) metres in sensor frame | None |
| Per-point timestamps for motion compensation | **DESIGNED but unpopulated** — `dt_ns` / `timestamp_offset_ns` fields exist, set to 0 on all production paths | Assembler should populate these |
| `PointCloudFrame` output | **EXISTS** — `odom::PointCloudFrame` is the correct output type | Needs `metadata` extension |

### 0.7 The hidden consequence: SLAM is broken on real hardware

The ESIKF in `AcmeSlamEngine` performs scan-to-map registration.  With
50–200 points per "scan" (one packet), the correspondence search has
virtually no geometric constraint.  ICP/ESIKF needs ~1,000+ well-distributed
points to converge reliably.  This means:

- **Simulated mode works** — full 5,760-point scans.
- **Real hardware is unusable** — 50–200 points per ESIKF update, called
  75× per revolution instead of once.  The ESIKF runs 75× more often
  than intended, with 75× fewer points per update, and each update
  mutates the state estimate.

The frame assembler is not merely a nice-to-have — it's a **prerequisite
for real hardware operation**.

---

## 1. Architecture

### 1.1 Where it sits

```
PacketParser / IPacketDecoder
        │
        │  lidar_cb_(LidarFrame)  — per-packet, 50–200 pts
        ▼
 ┌──────────────────────────────────────────────┐
 │           LidarFrameAssembler                │
 │                                              │
 │  Accumulate points + track azimuth coverage  │
 │  Detect revolution boundary (azimuth wrap)   │
 │  Handle packet drops (azimuth gaps)          │
 │  Timeout partial frames                      │
 │  Populate per-point dt_ns                    │
 │  Emit PointCloudFrame on completion          │
 └──────────────┬───────────────────────────────┘
                │
                │  scan_cb_(PointCloudFrame)  — per-revolution, ~28K pts
                ▼
        ┌───────────────────────┐
        │  Downstream consumers │
        │  • SlamTimeSync       │
        │  • TimeSyncEngine     │
        │  • SyncEngine         │
        │  • DataLayer          │
        │  • User callbacks     │
        └───────────────────────┘
```

### 1.2 Input/output contract

**Input:** Per-packet `LidarFrame` (from `PacketParser` or `IPacketDecoder`)
containing a sub-segment of one revolution.

**Output:** Per-revolution `PointCloudFrame` containing all points from
one complete or timed-out partial sweep, with populated per-point `dt_ns`
timestamps and assembly metadata.

### 1.3 What the assembler does NOT do

- **Deskewing** — that's `PointDeskewer` in the SLAM pipeline.  We provide
  raw points with `dt_ns` set so the deskewer can do its job.
- **Coordinate transformation** — points stay in device (sensor) frame.
- **Downsampling / ROI filtering** — that's the `PointCloudPreprocessor`.
- **Clock domain conversion** — that's `ClockService`.

---

## 2. Frame Completion Detection

### 2.1 The azimuth-wrap method

A spinning LiDAR's azimuth increases monotonically from 0° to 360° within
one revolution.  When azimuth wraps from ~360° back to ~0°, a new revolution
has begun.  All points accumulated so far constitute one complete frame.

```
Packet N:   azimuth_start = 348.5°,  azimuth_end = 352.0°
Packet N+1: azimuth_start = 352.0°,  azimuth_end = 356.0°
Packet N+2: azimuth_start = 356.0°,  azimuth_end = 359.5°
Packet N+3: azimuth_start =   0.5°,  azimuth_end =   4.0°   ← WRAP!
            ^^^^^^^^^^^^^^^^^^^^
            Revolution complete: emit all points from packets 0..N+2
            Start new accumulation with packet N+3
```

**Detection:** If the new packet's `azimuth_start` is **less than** the
previous packet's `azimuth_end` by more than a hysteresis threshold
(e.g. 180°), declare a wrap.  The 180° threshold prevents false triggers
from jitter or dual-return mode where azimuth might briefly decrease by
a few hundredths of a degree.

```cpp
bool is_wrap(float prev_end, float curr_start) {
    // If current start is numerically much less than previous end,
    // the sensor has wrapped past 360°.
    float delta = prev_end - curr_start;
    return delta > 180.0f;  // e.g., 358° → 2° gives delta = 356°
}
```

### 2.2 Why not sequence-number based?

The `PacketHeader::sequence` field increments per packet, not per
revolution.  There is no "scan ID" or "frame ID" in the protocol.
The VLP-16 has no per-packet sequence number at all.  Azimuth is the
only universal revolution boundary signal.

### 2.3 Edge case: solid-state LiDAR (non-spinning)

Solid-state LiDARs (Livox Mid-360, Hesai QT128) don't spin continuously.
They use a repetitive scan pattern that completes on a deterministic timer
(e.g., 100 ms for Livox Mid-360).  For these:

- Azimuth-wrap detection doesn't apply.
- Frame completion is **time-based**: emit a frame every `frame_period_ns`.
- The assembler supports both modes via `CompletionMode` config.

---

## 3. Interface Design

```cpp
// sdk/include/thunderbird/lidar_frame_assembler.h
#pragma once

#include "thunderbird/types.h"
#include "thunderbird/odom/slam_types.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace thunderbird {

// ── Completion detection mode ───────────────────────────────────────────────

enum class CompletionMode : uint8_t {
    /// Spinning LiDAR: detect revolution boundary via azimuth wrap.
    /// Works for Velodyne, Ouster, Hesai spinning models, etc.
    AzimuthWrap = 0,

    /// Solid-state / non-repetitive scan pattern: emit frame on a fixed
    /// timer.  frame_period_ns determines the accumulation window.
    TimeBased = 1,
};

// ── Configuration ───────────────────────────────────────────────────────────

struct FrameAssemblerConfig {
    CompletionMode mode{CompletionMode::AzimuthWrap};

    /// Expected LiDAR rotation rate in Hz.
    /// Used for:
    ///   - Partial-frame timeout calculation (2× expected period)
    ///   - Per-point dt_ns interpolation (scan duration estimate)
    ///   - Sanity checks on revolution timing
    /// Common values: 10.0 (most LiDARs), 20.0 (fast mode), 5.0 (slow).
    double expected_rate_hz{10.0};

    /// [TimeBased mode only] Fixed accumulation period (nanoseconds).
    /// Overrides expected_rate_hz for frame emission timing.
    /// Default matches 10 Hz.
    int64_t frame_period_ns{100'000'000};

    /// Maximum number of points per assembled frame.
    /// Safety cap to prevent runaway accumulation if wrap detection fails.
    /// Should be 2–3× the expected points/revolution.
    /// VLP-16: ~28,800/rev → cap at 60,000.
    /// OS1-64: ~131,072/rev → cap at 300,000.
    size_t max_points_per_frame{100'000};

    /// Maximum number of packets per frame (safety cap).
    uint32_t max_packets_per_frame{200};

    /// Partial-frame timeout multiplier.
    /// If no new packet arrives within (1/expected_rate_hz * timeout_mul),
    /// emit whatever we've accumulated as a partial frame.
    /// Default 2.0 = allow up to 2× the expected revolution period.
    double partial_timeout_mul{2.0};

    /// Minimum number of points for a frame to be emitted.
    /// Frames with fewer points are silently discarded (likely a runt at
    /// startup or after a reconnect).
    size_t min_points_to_emit{100};

    /// Azimuth wrap hysteresis (degrees).
    /// Jump in azimuth from the end of one packet to the start of the next
    /// that is >= this value (backwards) triggers a wrap.
    float azimuth_wrap_threshold_deg{180.0f};

    /// Pre-allocate point buffer to this capacity (avoids realloc on hot path).
    size_t point_reserve{32'768};
};

// ── Assembly metadata ───────────────────────────────────────────────────────

struct FrameAssemblyMeta {
    int64_t  scan_start_ns{0};           ///< HW timestamp of first packet
    int64_t  scan_end_ns{0};             ///< HW timestamp of last packet
    int64_t  scan_duration_ns{0};        ///< end − start
    int64_t  host_arrival_ns{0};         ///< host timestamp when frame emitted

    uint32_t total_packets{0};           ///< packets in this frame
    uint32_t expected_packets{0};        ///< estimated packets/revolution (from rate)

    float    azimuth_coverage_deg{0};    ///< total azimuth range covered
    float    azimuth_start_deg{0};       ///< first packet's azimuth_start
    float    azimuth_end_deg{0};         ///< last packet's azimuth_end

    uint32_t dropped_packets{0};         ///< estimated gaps (from azimuth discontinuity)
    double   drop_rate{0};               ///< dropped / (dropped + total)

    bool     is_partial{false};          ///< true if emitted due to timeout
    bool     is_first_frame{false};      ///< true for the first frame after reset
    uint32_t sequence{0};                ///< monotonic frame counter
};

// ── Output callback ─────────────────────────────────────────────────────────

/// Called once per assembled frame.
/// The PointCloudFrame is fully populated: timestamp_ns, points (with dt_ns),
/// sequence.
using AssembledFrameCallback = std::function<void(
    std::shared_ptr<const odom::PointCloudFrame> frame,
    const FrameAssemblyMeta& meta)>;

// ── Assembler statistics ────────────────────────────────────────────────────

struct AssemblerStats {
    uint64_t frames_emitted{0};          ///< total complete + partial frames
    uint64_t partial_frames{0};          ///< frames emitted due to timeout
    uint64_t packets_ingested{0};        ///< total packets fed in
    uint64_t points_ingested{0};         ///< total points across all packets
    uint64_t dropped_packets_total{0};   ///< cumulative detected drops
    uint64_t runt_frames_discarded{0};   ///< frames below min_points_to_emit
    double   avg_points_per_frame{0};    ///< rolling average
    double   avg_packets_per_frame{0};   ///< rolling average
    double   avg_frame_period_ms{0};     ///< rolling average inter-frame interval
    double   measured_rate_hz{0};        ///< 1000 / avg_frame_period_ms
};

// ─────────────────────────────────────────────────────────────────────────────
// LidarFrameAssembler — accumulates per-packet data into full 360° sweeps
// ─────────────────────────────────────────────────────────────────────────────

class LidarFrameAssembler {
public:
    explicit LidarFrameAssembler(FrameAssemblerConfig config = {});

    // ── Packet ingestion (called from I/O thread) ───────────────────────

    /// Feed ONE decoded LiDAR packet.
    ///
    /// @param frame           Per-packet LidarFrame from PacketParser
    /// @param azimuth_start   Start azimuth of this packet (degrees, 0–360)
    /// @param azimuth_end     End azimuth of this packet (degrees, 0–360)
    ///
    /// When a revolution boundary is detected, the accumulated frame is
    /// emitted via the registered callback before the new packet is stored.
    ///
    /// The azimuth parameters come from LidarSubHeader (Thunderbird native)
    /// or from the decoded firing blocks (VLP-16, Ouster, etc.).
    /// For decoders that don't provide azimuth, use feed_timed().
    void feed(std::shared_ptr<const LidarFrame> frame,
              float azimuth_start,
              float azimuth_end);

    /// Feed ONE decoded LiDAR packet (time-based mode).
    ///
    /// For solid-state LiDARs or sensors without azimuth data.
    /// The assembler accumulates points and emits a frame every
    /// frame_period_ns.
    void feed_timed(std::shared_ptr<const LidarFrame> frame);

    /// Check for partial-frame timeout.
    ///
    /// Call this periodically (e.g. from the I/O loop's idle path) to
    /// detect stalled sensors.  If the time since the last packet exceeds
    /// the timeout, emits a partial frame.
    ///
    /// @param now_ns  Current host timestamp (nanoseconds, steady_clock).
    void check_timeout(int64_t now_ns);

    // ── Output ──────────────────────────────────────────────────────────

    void on_frame(AssembledFrameCallback cb);

    // ── Diagnostics ─────────────────────────────────────────────────────

    [[nodiscard]] AssemblerStats stats() const;

    // ── Lifecycle ───────────────────────────────────────────────────────

    void reset();

private:
    FrameAssemblerConfig config_;
    // ... see §4 for internal state
};

} // namespace thunderbird
```

---

## 4. Internal State Machine

### 4.1 State diagram

```
                   feed(packet, az_start, az_end)
                              │
                              ▼
                   ┌────── state? ──────┐
                   │                    │
              COLLECTING          WAITING_FOR_FIRST
                   │                    │
                   ▼                    ▼
            ┌─────────────┐      ┌──────────────────┐
            │ Is azimuth  │      │ Store as first    │
            │ wrap?       │      │ packet of new     │
            │             │      │ accumulation.     │
            │ prev_end >  │      │ Transition to     │
            │ curr_start  │      │ COLLECTING.       │
            │ by > 180°?  │      └──────────────────┘
            └──┬──────┬───┘
            no │      │ yes
               ▼      ▼
       ┌───────────┐  ┌──────────────────────────┐
       │ Append    │  │ EMIT accumulated frame   │
       │ points    │  │  • Build PointCloudFrame  │
       │ to buffer │  │  • Populate dt_ns         │
       │           │  │  • Compute metadata       │
       │ Update    │  │  • Detect azimuth gaps    │
       │ azimuth   │  │  • Fire callback          │
       │ tracker   │  │                            │
       └───────────┘  │ Then: start new accum     │
                      │ with current packet       │
                      └──────────────────────────┘
```

### 4.2 Internal data (hot path — no heap allocation except point vector)

```cpp
struct LidarFrameAssembler::Impl {
    FrameAssemblerConfig config;

    // ── Accumulation buffer ─────────────────────────────────────────
    // Reused across frames: clear() doesn't deallocate.
    std::vector<odom::PointXYZIT> accum_points;
    
    // ── Per-packet metadata (for gap detection + dt_ns interpolation) ──
    struct PacketMeta {
        float    azimuth_start;
        float    azimuth_end;
        int64_t  hw_timestamp_ns;
        int64_t  host_timestamp_ns;
        uint32_t point_start_idx;   // index into accum_points
        uint32_t point_count;
    };
    std::vector<PacketMeta> packet_metas;

    // ── Revolution tracking ─────────────────────────────────────────
    float    prev_azimuth_end{0.0f};
    int64_t  first_packet_hw_ns{0};
    int64_t  last_packet_hw_ns{0};
    int64_t  last_packet_host_ns{0};
    uint32_t packets_in_frame{0};
    bool     have_first_packet{false};

    // ── Frame counter + timing ──────────────────────────────────────
    uint32_t frame_sequence{0};
    int64_t  prev_frame_emit_host_ns{0};  // for rate measurement

    // ── Statistics ──────────────────────────────────────────────────
    AssemblerStats stats;
    double sum_points_per_frame{0};
    double sum_packets_per_frame{0};
    double sum_frame_period_ms{0};
};
```

### 4.3 Key invariant: zero heap allocation on hot path

The `accum_points` vector is pre-reserved to `config.point_reserve` on
construction.  `clear()` sets size to 0 without freeing memory.  Subsequent
frames reuse the same allocation.  This is critical: at 10 Hz with 28K
points/frame, we cannot afford a 28K-element `std::vector` allocation per
revolution.

```cpp
LidarFrameAssembler::LidarFrameAssembler(FrameAssemblerConfig config)
    : impl_(std::make_unique<Impl>(config))
{
    impl_->accum_points.reserve(config.point_reserve);
    impl_->packet_metas.reserve(config.max_packets_per_frame);
}
```

---

## 5. Per-Point Timestamp (`dt_ns`) Population

### 5.1 The problem

Every downstream consumer that does motion compensation (deskewing) needs
to know *when* each point was acquired relative to the scan start.  The
`odom::PointXYZIT::dt_ns` field exists for this purpose but is set to 0
on all production paths.

### 5.2 Two strategies

**Strategy A: Packet-level interpolation (default)**

Assign each point a `dt_ns` based on linear interpolation between the
packet's hardware timestamp and the scan start timestamp.

```
                    scan_start_ns            scan_end_ns
                    │                        │
  Time: ──────────────────────────────────────────►
                    │   pkt0    pkt1   pkt2   │
                    │   ├──┤   ├──┤   ├──┤   │

  For each point in packet k:
      dt_ns = packet_k.hw_timestamp_ns − scan_start_ns
```

Within a single packet, all points get the same `dt_ns` (the packet's
HW timestamp relative to scan start).  This is sufficient for 68-packet
scans at 10 Hz: each packet spans ~1.47 ms (100 ms / 68), and intra-packet
motion is negligible for all but the most aggressive maneuvers.

**Strategy B: Intra-packet interpolation (optional, azimuth-based)**

For higher-fidelity deskewing, interpolate *within* each packet based on
the point's azimuth relative to the packet's azimuth range:

```
For point at azimuth θ within packet [az_start, az_end]
at HW time T_pkt, with scan duration T_scan:

    fraction = (θ − az_start) / (az_end − az_start)
    dt_ns = (T_pkt − scan_start_ns)
          + fraction × (T_pkt_next − T_pkt)
```

This requires computing `atan2(y, x)` per point (or storing the original
azimuth from the wire format).  Cost: ~5 ns/point.  Worthwhile for
high-speed drones but overkill for cars.

### 5.3 Implementation (Strategy A — used by default)

```cpp
void populate_dt_ns(std::vector<odom::PointXYZIT>& points,
                    const std::vector<PacketMeta>& metas,
                    int64_t scan_start_ns)
{
    for (const auto& m : metas) {
        int32_t dt = static_cast<int32_t>(m.hw_timestamp_ns - scan_start_ns);
        for (uint32_t i = m.point_start_idx;
             i < m.point_start_idx + m.point_count; ++i) {
            points[i].dt_ns = dt;
        }
    }
}
```

This is O(n) over points and runs in-place.  No allocation.

---

## 6. Packet Drop Detection

### 6.1 Azimuth gap analysis

After a revolution is complete, we have a sorted list of `PacketMeta`
entries with `[azimuth_start, azimuth_end]` ranges.  Ideally these are
contiguous:

```
Packet 0:  [  0.0°,   4.8°]
Packet 1:  [  4.8°,   9.6°]
Packet 2:  [  9.6°,  14.4°]
...
Packet 74: [355.2°, 360.0°]
```

A dropped packet creates a gap:

```
Packet 17: [ 81.6°,  86.4°]
  — MISSING packet(s) —
Packet 19: [ 96.0°, 100.8°]     ← gap: 86.4° → 96.0° = 9.6° missing
```

### 6.2 Gap detection algorithm

```cpp
uint32_t detect_drops(const std::vector<PacketMeta>& metas,
                      float expected_packet_span_deg)
{
    if (metas.size() < 2) return 0;
    uint32_t drops = 0;

    for (size_t i = 1; i < metas.size(); ++i) {
        float gap = metas[i].azimuth_start - metas[i-1].azimuth_end;
        // Normalise to [0, 360)
        if (gap < 0) gap += 360.0f;
        
        // If gap > 1.5× expected packet span, at least one packet was lost.
        if (gap > expected_packet_span_deg * 1.5f) {
            drops += static_cast<uint32_t>(
                gap / expected_packet_span_deg + 0.5f) - 1;
        }
    }
    return drops;
}
```

Expected packet span is estimated from the first few frames:
`360.0° / avg_packets_per_frame`.

### 6.3 What we do NOT do with drops

We do NOT attempt to interpolate missing points.  Azimuth gaps stay as
gaps in the point cloud.  Reasons:

1. Fabricated points would corrupt SLAM / obstacle detection.
2. The downstream preprocessor (ROI crop + RANSAC + clustering) is
   inherently robust to sparse regions.
3. The `drop_rate` in metadata lets consumers decide their own policy.

---

## 7. Partial-Frame Timeout

### 7.1 When it triggers

If `check_timeout(now_ns)` is called and:

```
now_ns − last_packet_host_ns  >  (1.0 / expected_rate_hz) × partial_timeout_mul
```

... the assembler emits whatever points it has accumulated as a partial frame.

### 7.2 Why it's necessary

- **Sensor disconnect**: packets stop arriving mid-revolution.  Without
  timeout, the accumulated points sit forever.
- **Sensor restart**: after a warm restart, the first packet's azimuth
  may not align with where the previous accumulation left off.  The
  timeout clears the stale buffer.
- **Network congestion**: a burst of packet loss at revolution boundaries
  can prevent wrap detection.

### 7.3 Timeout value

At 10 Hz rotation (100 ms/revolution): timeout = 200 ms (2× period).

This is generous enough to survive one dropped wrap-boundary packet
(the next wrap at 100 ms later will trigger normally) but tight enough
to not accumulate stale data for more than one revolution period.

---

## 8. Configurable Frame Rate

### 8.1 Spinning LiDARs: rate = rotation speed

For spinning sensors, the frame rate is determined by the motor speed,
not by the SDK.  The assembler's `expected_rate_hz` is an *estimate*
used for timeout calculation and diagnostics.  The actual frame rate
is whatever the sensor delivers.

Some sensors support runtime rotation-speed changes (e.g., VLP-16: 5/10/20 Hz).
The assembler handles this transparently because it detects revolution
boundaries from azimuth wraps, not from timing.

The assembler reports `measured_rate_hz` in its statistics, which the
user can compare to `expected_rate_hz` to detect misconfiguration.

### 8.2 Solid-state LiDARs: rate = configured period

For `CompletionMode::TimeBased`, the assembler emits frames every
`frame_period_ns` nanoseconds.  This is a hard timer: points are
accumulated during the window, then emitted and the buffer cleared.

### 8.3 Rate change detection

If the measured rate deviates from expected by > 50%, the assembler
updates its internal `expected_packet_span_deg` estimate (used by drop
detection) to match reality.  This adaptive behavior handles:

- Runtime rotation speed changes
- Clock drift between host and sensor
- Sensors that don't rotate at exactly the advertised rate

---

## 9. Frame Emission (Hot Path)

### 9.1 Pseudocode

```cpp
void emit_frame(bool is_partial) {
    if (accum_points.size() < config.min_points_to_emit) {
        ++stats.runt_frames_discarded;
        clear_accumulator();
        return;
    }

    // ── Build PointCloudFrame ─────────────────────────────────────────
    auto cloud = std::make_shared<odom::PointCloudFrame>();
    cloud->timestamp_ns = first_packet_hw_ns;
    cloud->sequence     = frame_sequence++;
    cloud->is_deskewed  = false;

    // Move points (steals the vector's contents — zero copy).
    // After this, accum_points is empty but retains its allocation.
    cloud->points = std::move(accum_points);

    // ── Populate per-point dt_ns ──────────────────────────────────────
    populate_dt_ns(cloud->points, packet_metas, first_packet_hw_ns);

    // ── Build metadata ────────────────────────────────────────────────
    FrameAssemblyMeta meta;
    meta.scan_start_ns    = first_packet_hw_ns;
    meta.scan_end_ns      = last_packet_hw_ns;
    meta.scan_duration_ns = last_packet_hw_ns - first_packet_hw_ns;
    meta.host_arrival_ns  = last_packet_host_ns;
    meta.total_packets    = packets_in_frame;
    meta.is_partial       = is_partial;
    meta.is_first_frame   = (frame_sequence == 1);
    meta.sequence         = cloud->sequence;

    // Azimuth coverage
    if (!packet_metas.empty()) {
        meta.azimuth_start_deg = packet_metas.front().azimuth_start;
        meta.azimuth_end_deg   = packet_metas.back().azimuth_end;
        meta.azimuth_coverage_deg = compute_coverage(packet_metas);
    }

    // Drop detection
    float expected_span = (stats.avg_packets_per_frame > 0)
        ? 360.0f / static_cast<float>(stats.avg_packets_per_frame)
        : 360.0f / static_cast<float>(packets_in_frame);
    meta.dropped_packets = detect_drops(packet_metas, expected_span);
    meta.expected_packets = static_cast<uint32_t>(
        stats.avg_packets_per_frame > 0
            ? stats.avg_packets_per_frame
            : packets_in_frame);
    meta.drop_rate = (meta.dropped_packets + meta.total_packets > 0)
        ? static_cast<double>(meta.dropped_packets) /
          static_cast<double>(meta.dropped_packets + meta.total_packets)
        : 0.0;

    // ── Update statistics ─────────────────────────────────────────────
    update_stats(meta);

    // ── Reclaim buffer (accum_points was moved; re-reserve) ───────────
    accum_points.reserve(config.point_reserve);
    packet_metas.clear();
    packets_in_frame = 0;
    have_first_packet = false;

    // ── Emit ──────────────────────────────────────────────────────────
    if (frame_cb_) {
        frame_cb_(std::move(cloud), meta);
    }
}
```

### 9.2 Latency analysis

| Operation | Cost | Notes |
|-----------|------|-------|
| `std::move(accum_points)` | O(1) | Just pointer swap |
| `populate_dt_ns()` | O(n) | ~28K points × 1 ns/point = ~28 μs |
| `detect_drops()` | O(m) | ~75 packets × 10 ns/packet = ~750 ns |
| `make_shared<PointCloudFrame>` | 1 alloc | ~50 ns (tcmalloc) |
| `accum_points.reserve()` | 0–1 alloc | Usually reuses moved-from vector's allocation¹ |
| Total | | **< 50 μs** |

¹ After `std::move`, `accum_points` is in a valid-but-unspecified state.
Some implementations leave the capacity intact; others don't.  The
explicit `reserve()` handles both cases.

---

## 10. Integration Points

### 10.1 PacketParser integration

```cpp
// In PacketParser::dispatch_lidar() — after constructing the LidarFrame:

void dispatch_lidar(/* ... */) {
    // ... existing point extraction ...

    if (frame_assembler_) {
        // Forward to assembler instead of directly to user callback.
        frame_assembler_->feed(
            frame,
            sub.azimuth_start,
            sub.azimuth_end);
    } else {
        // Legacy path: fire per-packet callback (backward compat).
        if (lidar_cb_) lidar_cb_(std::move(frame));
    }
}
```

### 10.2 VelodyneVlp16Decoder integration

The decoder already tracks `first_azimuth` and `last_azimuth` per packet.
Integration is identical:

```cpp
void parse_data_packet(const uint8_t* pkt) {
    // ... existing parsing ...

    // Instead of lidar_cb_(frame):
    if (frame_assembler_) {
        frame_assembler_->feed(frame, first_azimuth, last_azimuth);
    } else {
        if (lidar_cb_) lidar_cb_(std::move(frame));
    }
}
```

### 10.3 DeviceManager wiring

```cpp
// In DeviceManager::connect():

// Before: parser calls lidar_cb directly with per-packet frames.
// After:  parser → assembler → lidar_cb with per-revolution frames.

impl_->frame_assembler = std::make_unique<LidarFrameAssembler>(
    config.frame_assembler);

impl_->frame_assembler->on_frame(
    [this](std::shared_ptr<const odom::PointCloudFrame> cloud,
           const FrameAssemblyMeta& meta) {
        // Feed full-revolution cloud to downstream
        time_sync->feed_lidar(cloud, meta.host_arrival_ns);
        // ... also feed SyncEngine, DataLayer, user callbacks
    });
```

### 10.4 SlamDaemon wiring

Currently does:
```cpp
d.device->on_lidar([&d](std::shared_ptr<const LidarFrame> f) {
    auto cloud = std::make_shared<odom::PointCloudFrame>();
    cloud->points.reserve(f->points.size());
    for (const auto& p : f->points) {
        cloud->points.push_back({p.x, p.y, p.z, p.intensity, 0});
    }
    d.engine->feedPointCloud(std::move(cloud), host_ns);
});
```

After integration, this entire callback is **deleted**.  The assembler
emits `odom::PointCloudFrame` directly — no per-point conversion needed
because the assembler already produces `odom::PointXYZIT` with populated
`dt_ns`.

---

## 11. Thread Safety

The assembler is designed for **single-producer** use on the I/O thread.
All `feed()` and `check_timeout()` calls come from the same thread that
runs `PacketParser::feed()`.

The output callback (`frame_cb_`) is invoked synchronously from `feed()`.
If the consumer needs thread crossing (e.g., pushing to a ring buffer for
the ESIKF thread), the callback does that — the assembler doesn't.

**No mutexes in the assembler.** The entire hot path is lock-free.

---

## 12. Timeout Polling

### 12.1 Where to call `check_timeout()`

The I/O loop already has a read timeout (waiting for the next transport
chunk).  When the read times out with zero bytes, call:

```cpp
// In the I/O loop:
size_t n = transport.read(buf, sizeof(buf), timeout_ms);
if (n > 0) {
    parser.feed(buf, n);
} else {
    // No data arrived — check for partial frame timeout.
    auto now_ns = Timestamp::now().nanoseconds;
    frame_assembler.check_timeout(now_ns);
}
```

This piggybacks on the existing transport polling without adding a
separate timer or thread.

---

## 13. File Changes Summary

| File | Change | New/Modify |
|------|--------|-----------|
| `sdk/include/thunderbird/lidar_frame_assembler.h` | Full class: `LidarFrameAssembler`, `FrameAssemblerConfig`, `FrameAssemblyMeta`, `AssemblerStats` | **New** |
| `sdk/src/lidar_frame_assembler.cpp` | Implementation: accumulation, azimuth wrap detection, emit, timeout, dt_ns population, drop detection | **New** |
| `sdk/include/thunderbird/packet_parser.h` | Accept optional `LidarFrameAssembler*`. In `dispatch_lidar()`, feed assembler instead of direct callback when present. | Modify |
| `sdk/src/device_manager.cpp` | Own `LidarFrameAssembler`. Wire it between parser and downstream consumers. Delete the per-packet → PointCloudFrame conversion. | Modify |
| `sdk/include/thunderbird/device_manager.h` | Add `FrameAssemblerConfig` to `DeviceConfig`. | Modify |
| `slamd/src/acme_slamd.cpp` | Delete the `on_lidar` callback that manually converts LidarFrame → PointCloudFrame. Wire `on_frame` from assembler → engine. | Modify |
| `tests/test_frame_assembler.cpp` | Unit tests: revolution detection, drop handling, timeout, dt_ns, runt rejection, rate measurement. | **New** |

### 13.1 Implementation order

1. **`lidar_frame_assembler.h/cpp`** — standalone, no existing code dependencies beyond types
2. **Unit tests** — validate in isolation with synthetic packets
3. **`PacketParser` integration** — add assembler injection point
4. **`DeviceManager` wiring** — own assembler, wire callbacks
5. **`SlamDaemon` cleanup** — remove manual per-packet conversion
6. **VelodyneVlp16Decoder integration** — wire azimuth to assembler

---

## 14. What This Design Does NOT Include

| Item | Reason |
|------|--------|
| Multi-return handling | VLP-16 dual-return mode requires per-decoder logic (decode both returns); the assembler sees points regardless of return mode |
| Point deskewing | Downstream responsibility (`PointDeskewer`); we provide `dt_ns` so it can do its job |
| Coordinate transforms | The assembler passes through points in device (sensor) frame; extrinsic calibration is applied downstream |
| Voxel downsampling | That's the `PointCloudPreprocessor`; the assembler emits raw, full-density clouds |
| Multi-LiDAR assembly | One assembler per LiDAR sensor; multi-sensor fusion is a separate concern |
| Ring buffer output | The assembler fires a callback; if the consumer needs a ring buffer (e.g., for the ESIKF thread), it wraps the callback — the assembler doesn't prescribe the threading model |
