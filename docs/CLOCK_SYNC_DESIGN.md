# Thunderbird SDK — Hardware Timestamp Synchronization Design

> **Status:** Design proposal  
> **Author:** Timing Systems Architecture  
> **Date:** 2026-03-01  
> **Depends on:** `ClockDriftModel`, `SlamTimeSync`, `TimeSyncEngine`, `SyncEngine`, `PacketParser`, `protocol.h`

---

## 0. Existing Code Audit

The codebase already has **three** time synchronization subsystems and a
clock-drift estimator.  Before designing anything, the audit must be precise.

### 0.1 What already exists

| Component | File | What it does | Drift estimation? | Timestamp domain awareness? |
|-----------|------|-------------|-------------------|---------------------------|
| **`SyncEngine`** | `sync_engine.h` | Nearest-neighbour LiDAR↔IMU↔Camera bundle alignment. Uses `types.h` timestamp. | No | No |
| **`TimeSyncEngine`** | `time_sync.h` | LiDAR-driven sync with IMU block collection + camera tolerance. OLS drift on lidar↔camera offset. | **Yes** — slope of (lidar_ts, lidar↔camera_offset) | No — operates on `sensor_data.h` hardware timestamps only |
| **`SlamTimeSync`** | `slam_time_sync.h` | SLAM-specific: gap-free IMU block with boundary interpolation, out-of-order reordering, clock drift compensation. | **Yes** — full `ClockDriftModel` (α, β linear regression) | **Yes** — `ClockDomain::Hardware` / `Host`, `resolve_timestamp()` |
| **`ClockDriftModel`** | `slam_time_sync.h` (inline) | Sliding-window OLS: `host = α·hw + β`. Converts between clock domains. Reports drift in ns/s. | **This IS the estimator** | Yes — bidirectional conversion |
| **`PacketParser`** | `packet_parser.h` | Stamps `host_timestamp = Timestamp::now()` at decode time. Extracts `hw_timestamp_ns` from packet header. | No | Implicitly — produces both timestamps |
| **`HeartbeatPayload`** | `protocol.h` | Carries `sender_timestamp_ns` for round-trip latency estimation. | No (unused for clock sync) | N/A |
| **`PacketHeader`** | `protocol.h` | 8-byte `hw_timestamp_ns` in every packet — device-side nanosecond timestamp. | N/A | Hardware domain |
| **`Timestamp`** | `types.h` | `steady_clock::now()` wrapper. Comment says "Epoch configurable (defaults to system boot / PTP epoch)" but **no PTP or epoch configuration exists**. | N/A | Host domain only |

### 0.2 What's missing (actual gaps)

| Requirement | Status | Gap |
|-------------|--------|-----|
| Recover hardware time | **Done** — `PacketHeader::hw_timestamp_ns` extracted by `PacketParser` | None |
| Estimate offset to system clock | **Partially done** — `ClockDriftModel` does `host = α·hw + β` but with `Timestamp::now()` jitter (10–100 μs). No kernel RX timestamps. | Need `SO_TIMESTAMPING` integration (designed in PHASE2_TRANSPORT_DESIGN.md) to reduce jitter to <1 μs |
| Compensate drift | **Done** in `SlamTimeSync` path only. `TimeSyncEngine` estimates drift slope but **never compensates** — only warns. `SyncEngine` ignores drift entirely. | Need to propagate drift compensation to `TimeSyncEngine` and/or unify |
| Unified timestamp to SDK core | **Not done** — three sync engines, two timestamp types (`Timestamp` vs raw `int64_t`), no single clock authority | Need a `ClockService` that all subsystems query |
| PPS signal handling | **Not implemented** — not referenced anywhere in code | Full gap |
| PTP support | **Not implemented** — `types.h:19` mentions "PTP epoch" in a comment but no code exists | Full gap — but PTP is an OS-level concern, not ours (see §3.3) |
| Time jump detection | **Not implemented** — `ClockDriftModel::recompute()` does no outlier rejection | Full gap |
| Car profile (GNSS available) | **Config exists** in perception — `MotionModel::CTRV` for car — but no GNSS-disciplined clock | Full gap |
| Drone profile (no GNSS) | **Config exists** — `MotionModel::ConstantVelocity` for drone — but no free-running clock management | Full gap |
| host_timestamp quality | **Done but low quality** — `Timestamp::now()` at `dispatch()` time in parser has scheduling jitter | Addressed by SO_TIMESTAMPING design |

### 0.3 Architectural problem: three sync engines

```
Device → PacketParser → LidarFrame (timestamp + host_timestamp)
                               │
              ┌────────────────┼────────────────────┐
              ▼                ▼                     ▼
         SyncEngine      TimeSyncEngine         SlamTimeSync
         (types.h)       (sensor_data.h)        (slam_types.h)
              │                │                     │
         SyncBundle       SyncedFrame         ScanMeasurement
              │                │                     │
         User callback    Pull/Callback API     ESIKF thread
```

Three subsystems, three timestamp domains, no shared clock authority.
`SyncEngine` has no drift awareness at all.  `TimeSyncEngine` estimates
drift but doesn't compensate.  `SlamTimeSync` compensates but only for
the ESIKF path.  If the device clock drifts 5 ms/s (common for cheap
MEMS oscillators), the non-SLAM paths get progressively worse alignment.

**This is the core problem to solve.**

---

## 1. Design: `ClockService` — Unified Clock Authority

### 1.1 Concept

A single, shared `ClockService` replaces the scattered drift-tracking
logic.  All timestamp consumers query it instead of maintaining their own
models.

```
                          ┌───────────────────────────────┐
                          │         ClockService           │
                          │                                │
                          │  ClockOffsetEstimator          │
                          │    (OLS + outlier rejection)   │
                          │                                │
                          │  PpsAnchor (optional)          │
                          │    (1PPS → absolute epoch)     │
                          │                                │
                          │  JumpDetector                  │
                          │                                │
                          │  ClockProfile (car/drone)      │
                          │                                │
                          └───────┬───────────────────────┘
                                  │ unified_timestamp(hw_ns)
                                  │ hw_to_host() / host_to_hw()
                     ┌────────────┼─────────────────┐
                     ▼            ▼                  ▼
               SyncEngine   TimeSyncEngine      SlamTimeSync
                 (uses)        (uses)              (uses)
```

**What changes:** `SlamTimeSync::ClockDriftModel` gets extracted to
`ClockService` and shared.  `TimeSyncEngine` stops doing its own drift
regression.  `SyncEngine` learns to call `unified_timestamp()`.

**What doesn't change:** The existing `ClockDriftModel` algorithm is sound
(sliding-window OLS).  We reuse it as-is, adding outlier rejection and
jump detection on top.

### 1.2 Interface

```cpp
// sdk/include/thunderbird/clock_service.h
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>

namespace thunderbird {

// ── Clock synchronization profile ───────────────────────────────────────────

enum class ClockProfile : uint8_t {
    /// Car: GNSS-disciplined device clock.  Expect sub-μs HW timestamps
    /// with occasional jump (leap second, GNSS reacquisition).
    /// PPS available.  Drift is typically < 100 ns/s.
    Car = 0,

    /// Drone: Free-running crystal oscillator on device.  No absolute
    /// time reference.  Drift 1–50 μs/s typical (MEMS XO).
    /// No PPS.  Must rely on host↔device offset estimation only.
    Drone = 1,
};

// ── Configuration ───────────────────────────────────────────────────────────

struct ClockServiceConfig {
    ClockProfile profile{ClockProfile::Drone};  // default = worst case

    /// OLS window size for offset estimation.
    size_t ols_window{100};

    /// Jump detection threshold (ns).  If a single observation deviates
    /// from the model prediction by more than this, it's flagged as a
    /// potential time jump rather than a normal sample.
    int64_t jump_threshold_ns{50'000'000};  // 50 ms

    /// Number of consecutive jump-flagged samples before declaring a
    /// confirmed time jump (as opposed to transient noise).
    uint32_t jump_confirm_count{5};

    /// Maximum acceptable drift rate (ns/s) before warning.
    double drift_warn_threshold_ns_per_sec{1'000'000.0};  // 1 ms/s

    /// Enable PPS anchoring (Car profile only; ignored if no PPS).
    bool enable_pps{false};

    /// PPS expected interval in nanoseconds (nominally 1 second).
    int64_t pps_interval_ns{1'000'000'000};

    /// PPS tolerance: max jitter before a PPS pulse is rejected.
    int64_t pps_tolerance_ns{10'000'000};  // 10 ms
};

// ── Diagnostic events ───────────────────────────────────────────────────────

enum class ClockEvent : uint8_t {
    Calibrated,         ///< first valid offset estimate available
    DriftWarning,       ///< drift exceeds threshold
    TimeJump,           ///< confirmed time jump detected, model reset
    PpsLocked,          ///< PPS anchor acquired
    PpsLost,            ///< PPS signal lost (N misses)
    ModelReset,         ///< model was reset (e.g. after jump or reconnect)
};

struct ClockDiagnostics {
    double   offset_ns{0};           ///< current estimated offset (host − hw)
    double   drift_ns_per_sec{0};    ///< current drift rate
    double   offset_stddev_ns{0};    ///< uncertainty of offset estimate
    uint64_t observations{0};        ///< total (hw, host) pairs ingested
    uint64_t jumps_detected{0};      ///< total confirmed time jumps
    uint64_t outliers_rejected{0};   ///< samples rejected by outlier filter
    bool     calibrated{false};      ///< true once model has enough samples
    bool     pps_locked{false};      ///< true if PPS anchor is active
};

using ClockEventCallback = std::function<void(ClockEvent, const ClockDiagnostics&)>;

// ─────────────────────────────────────────────────────────────────────────────
// ClockService — THE single clock authority for the entire SDK
// ─────────────────────────────────────────────────────────────────────────────

class ClockService {
public:
    explicit ClockService(ClockServiceConfig config = {});

    // ── Observation ingestion (called from I/O thread) ──────────────────

    /// Record a (hardware_timestamp, host_timestamp) pair.
    ///
    /// This is the fundamental input.  Called for every decoded packet
    /// (or at a decimated rate for high-frequency sensors like IMU).
    ///
    /// @param hw_ns    Hardware timestamp from device (PacketHeader::hw_timestamp_ns)
    /// @param host_ns  Host-side timestamp.  Prefer kernel RX timestamp
    ///                 (SO_TIMESTAMPING) when available; fall back to
    ///                 steady_clock::now().
    void observe(int64_t hw_ns, int64_t host_ns);

    /// Record a PPS edge arrival (Car profile only).
    ///
    /// @param host_ns  Host timestamp of PPS rising edge (ideally from
    ///                 GPIO interrupt handler or kernel PPS subsystem).
    /// @param pps_hw_ns  (Optional) If the device reports the HW timestamp
    ///                   corresponding to this PPS edge, pass it here.
    ///                   Pass 0 if unknown.
    void observe_pps(int64_t host_ns, int64_t pps_hw_ns = 0);

    // ── Timestamp conversion (called from any thread) ───────────────────

    /// Convert hardware timestamp to unified host clock domain.
    /// Before calibration, returns hw_ns unchanged (identity).
    /// After calibration, applies the OLS model: host = α·hw + β.
    ///
    /// Thread-safe: reads atomic snapshot of model parameters.
    [[nodiscard]] int64_t hw_to_host(int64_t hw_ns) const noexcept;

    /// Convert host timestamp to hardware clock domain (inverse).
    [[nodiscard]] int64_t host_to_hw(int64_t host_ns) const noexcept;

    /// Return the best unified timestamp for a given (hw, host) pair.
    /// - If calibrated: returns hw_to_host(hw_ns) (more stable than host_ns)
    /// - If not calibrated: returns host_ns (best we have)
    ///
    /// This is the ONE function that all sync engines should call.
    [[nodiscard]] int64_t unified_timestamp(int64_t hw_ns, int64_t host_ns) const noexcept;

    // ── Diagnostics ─────────────────────────────────────────────────────

    [[nodiscard]] ClockDiagnostics diagnostics() const;
    [[nodiscard]] bool is_calibrated() const noexcept;
    [[nodiscard]] double drift_ns_per_sec() const noexcept;

    void on_event(ClockEventCallback cb);

    // ── Lifecycle ───────────────────────────────────────────────────────

    void reset();

private:
    // See §2 for algorithm details
    ClockServiceConfig config_;
    // ... internal state ...
};

} // namespace thunderbird
```

### 1.3 Why one service, not per-engine models

- **Consistency**: all subsystems agree on the current offset.  Today,
  `SlamTimeSync` and `TimeSyncEngine` independently estimate drift and
  can diverge.
- **Sample efficiency**: IMU at 400 Hz + LiDAR at 10 Hz + Camera at 30 Hz
  = 440 observations/second feeding one model, vs. each engine seeing
  only its subset.
- **Jump detection requires global view**: a time jump affects all sensors
  simultaneously.  Per-engine models detect it at different times with
  different latencies.
- **PPS anchoring is inherently global**: a PPS pulse disciplines the
  entire clock model, not individual engines.

---

## 2. `ClockOffsetEstimator` — Core Algorithm

### 2.1 Reuse of `ClockDriftModel`

The existing `ClockDriftModel` in `slam_time_sync.h` is **algorithmically
correct**.  Its OLS implementation:

```
host_ts = α · hw_ts + β
```

where α ≈ 1.0 encodes drift rate and β encodes the fixed offset, is exactly
what we need.  The drift rate in ns/s is `(α − 1) × 10⁹`.

**What we add on top:**

1. Outlier rejection (the existing model has none)
2. Jump detection
3. Residual-based uncertainty estimation
4. Lock-free read path (existing model isn't thread-safe for reads)

### 2.2 Outlier rejection

A single bad host timestamp (scheduling jitter spike, context switch,
GC pause in a co-located process) can corrupt the OLS model for an
entire window.  The fix is classic: **Median Absolute Deviation (MAD)**.

```
For each new observation (hw, host):
  1. predicted_host = α·hw + β    (from current model)
  2. residual = host − predicted_host
  3. If model is calibrated AND |residual| > k × MAD:
       → reject observation, increment outliers_rejected
       → BUT also increment jump_candidate_count
  4. Else:
       → accept into OLS window, reset jump_candidate_count
```

`k = 3.0` (3σ equivalent for Gaussian) is a reasonable default.  MAD
is maintained as a running median of the last N residuals using a
fixed-size sorted array (no heap allocation; N = window size).

### 2.3 Time jump detection

A time jump is fundamentally different from an outlier: it's a **permanent
step change** in the clock offset.  Device firmware updates, GNSS
reacquisition, NTP step adjustments, or sensor power cycles all cause jumps.

**Detection:**

```
If jump_candidate_count >= jump_confirm_count:
    → This is NOT noise; the clock actually jumped.
    → Record the jump magnitude (median of recent rejected residuals).
    → Reset the OLS model.
    → Emit ClockEvent::TimeJump.
    → Re-seed the model with the last jump_confirm_count observations
      (which represent the NEW clock state).
```

**Why a confirmation count?**  A single large residual could be network
jitter.  Five consecutive large residuals (at 10–440 Hz, that's 11–500 ms)
is a real clock event.

**Car vs. drone behavior:**

| Scenario | Car (GNSS) | Drone (free-running) |
|----------|-----------|---------------------|
| GNSS reacquisition | Jump when receiver re-locks after tunnel/building. Typically < 1 s magnitude. | N/A |
| Firmware timestamp reset | Device reboots mid-stream. Jump = full offset. | Same |
| NTP/PTP step | Host clock step. Rarely > 100 ms. | Same |
| Leap second | 1 s jump in UTC. GNSS reports it. | N/A |
| Crystal aging | N/A (GNSS corrects) | Slow drift, never a jump |

For the drone profile, jumps are rare (only device reboot).  For car,
they're expected during GNSS transitions.

### 2.4 State machine

```
                  observe(hw, host)
                        │
                        ▼
              ┌───── calibrated? ─────┐
              │ no                    │ yes
              ▼                       ▼
        ┌───────────┐        ┌───────────────────┐
        │ COLLECTING │        │ COMPUTE RESIDUAL  │
        │ Add to OLS │        │ r = host - predict│
        │ window     │        └────────┬──────────┘
        │            │                 │
        │ n >= 3?    │       ┌─────────┴──────────┐
        │  yes → fit │       │ |r| > k·MAD ?      │
        │  emit      │       └──┬─────────────┬───┘
        │ Calibrated │       no │             │ yes
        └───────────┘          ▼              ▼
                       ┌──────────────┐  ┌──────────────────┐
                       │ ACCEPT       │  │ REJECT (outlier)  │
                       │ Add to OLS   │  │ outliers_rejected++│
                       │ Recompute    │  │ jump_candidates++ │
                       │ Reset jump   │  │                   │
                       │  counter     │  │ candidates >=     │
                       └──────────────┘  │  confirm_count?   │
                                         └───┬──────────┬────┘
                                          no │          │ yes
                                             ▼          ▼
                                        ┌────────┐  ┌──────────────────┐
                                        │ WAIT   │  │ TIME JUMP        │
                                        │ (next  │  │ Reset OLS model  │
                                        │  obs)  │  │ Re-seed with     │
                                        └────────┘  │  recent rejects  │
                                                    │ Emit TimeJump    │
                                                    │ jump_candidates=0│
                                                    └──────────────────┘
```

### 2.5 Lock-free read path

`hw_to_host()` and `unified_timestamp()` will be called from multiple
threads (I/O thread, sync engine thread, ESIKF thread, user callback
threads).  The write path (`observe()`) runs on the I/O thread only.

We use a **seqlock** pattern: the writer increments a sequence counter
before and after updating (α, β, calibrated).  Readers spin-retry if
they observe an odd sequence or a torn read:

```cpp
// Writer (single thread — I/O loop):
void publish_model(double alpha, double beta, bool valid) {
    seq_.fetch_add(1, std::memory_order_release);  // odd = writing
    alpha_.store(alpha, std::memory_order_relaxed);
    beta_.store(beta, std::memory_order_relaxed);
    calibrated_.store(valid, std::memory_order_relaxed);
    seq_.fetch_add(1, std::memory_order_release);  // even = safe
}

// Reader (any thread):
int64_t hw_to_host(int64_t hw_ns) const noexcept {
    double a, b;
    bool cal;
    uint32_t s;
    do {
        s = seq_.load(std::memory_order_acquire);
        if (s & 1) continue;   // write in progress — spin
        a   = alpha_.load(std::memory_order_relaxed);
        b   = beta_.load(std::memory_order_relaxed);
        cal = calibrated_.load(std::memory_order_relaxed);
    } while (seq_.load(std::memory_order_acquire) != s);

    if (!cal) return hw_ns;
    return static_cast<int64_t>(a * static_cast<double>(hw_ns) + b);
}
```

This is zero-allocation, zero-contention, and wait-free on the read side
under SPSC (which is our normal operating mode; multi-reader contention
is benign — just retry).

### 2.6 Uncertainty estimation

After each OLS recompute, calculate the standard deviation of residuals:

```
σ² = Σ(residual²) / (n − 2)
```

This gives `offset_stddev_ns` in `ClockDiagnostics`, which tells
downstream consumers how much to trust the converted timestamp.

| Quality tier | σ | Typical cause |
|---|---|---|
| Excellent | < 1 μs | SO_TIMESTAMPING + stable network |
| Good | 1–100 μs | Software timestamps, low jitter |
| Degraded | 100 μs – 1 ms | High CPU load, USB transport |
| Unusable | > 1 ms | Broken clock, PPS lost, jump in progress |

---

## 3. PPS Anchoring (Car Profile)

### 3.1 What PPS gives us

A 1PPS (one pulse-per-second) signal from the GNSS receiver provides
an **absolute time mark**: the rising edge occurs at the exact UTC second
boundary.  This doesn't replace the offset estimator — it **anchors** it.

The offset estimator alone can tell us "the device clock runs 5 ppm fast
relative to the host."  But it can't tell us which clock is *right* in
absolute terms.  PPS tells us which second boundary is *now*, giving us
an absolute epoch.

### 3.2 PPS integration model

```
    GNSS Receiver
         │
         │  1PPS (GPIO / kernel PPS subsystem)
         ▼
    ┌──────────────────────────────────────────┐
    │  ClockService::observe_pps(host_ns)      │
    │                                          │
    │  1. Validate interval: |Δt − 1s| < tol  │
    │  2. Quantize host_ns to nearest second:  │
    │     pps_epoch = round(host_ns / 1e9) × 1e9│
    │  3. Compute PPS offset:                  │
    │     pps_offset = pps_epoch − host_ns     │
    │  4. Apply as correction to OLS model β   │
    │  5. After N consistent pulses → PpsLocked│
    └──────────────────────────────────────────┘
```

**Why we don't replace OLS with PPS:**  PPS gives us one sample per
second.  Between pulses, the OLS model handles interpolation and
drift compensation at full sensor rate.  PPS just prevents the OLS
offset from slowly walking away from absolute time.

### 3.3 PTP: NOT our problem

PTP (IEEE 1588) is an **operating-system and network-infrastructure
concern**, not an SDK concern.  A properly configured Linux system with
`ptp4l` + `phc2sys` disciplines `CLOCK_REALTIME` to the PTP grandmaster.
The SDK's `SO_TIMESTAMPING` (Phase 2 Transport Design) already captures
kernel RX timestamps in this disciplined domain.

What the SDK needs to do for PTP:
- Use `CLOCK_REALTIME` instead of `CLOCK_MONOTONIC`/`steady_clock` as
  the host timestamp source when PTP is active.
- Detect when PTP is available (check `sysfs` or `ethtool` PHC status).

What the SDK does NOT need to do:
- Implement PTP protocol
- Run a PTP slave
- Parse PTP announce/sync/follow-up packets

This is a **one-line configuration change**, not an architecture component:

```cpp
// In ClockServiceConfig:
enum class HostClockSource : uint8_t {
    SteadyClock    = 0,  // CLOCK_MONOTONIC — default, no absolute time
    RealtimeClock  = 1,  // CLOCK_REALTIME  — use when PTP/NTP disciplined
    KernelRxStamp  = 2,  // SO_TIMESTAMPING — best quality, when available
};

HostClockSource host_clock{HostClockSource::SteadyClock};
```

---

## 4. Car vs. Drone Profile Behavior

### 4.1 Configuration differences

```cpp
// Car profile defaults:
ClockServiceConfig car_config() {
    return {
        .profile = ClockProfile::Car,
        .ols_window = 200,                        // longer window — GNSS provides stable base
        .jump_threshold_ns = 10'000'000,          // 10 ms — tighter (GNSS is accurate)
        .jump_confirm_count = 3,                  // faster confirmation (GNSS jumps are real)
        .drift_warn_threshold_ns_per_sec = 100,   // 100 ns/s — GNSS-disciplined clocks barely drift
        .enable_pps = true,
    };
}

// Drone profile defaults:
ClockServiceConfig drone_config() {
    return {
        .profile = ClockProfile::Drone,
        .ols_window = 50,                          // shorter window — adapt faster to drift
        .jump_threshold_ns = 50'000'000,           // 50 ms — looser (no absolute reference)
        .jump_confirm_count = 10,                  // more conservative (don't false-trigger)
        .drift_warn_threshold_ns_per_sec = 1'000'000,  // 1 ms/s — MEMS XO can drift a lot
        .enable_pps = false,
    };
}
```

### 4.2 Behavioral differences

| Aspect | Car (GNSS) | Drone (no GNSS) |
|--------|-----------|-----------------|
| **Dominant error source** | GNSS reacquisition jumps after signal loss | Continuous crystal drift (temperature-dependent) |
| **Drift rate** | < 100 ns/s (GNSS-disciplined TCXO) | 1–50 μs/s (MEMS XO, varies with temperature) |
| **OLS window** | 200 samples (≈20 s at 10 Hz) — long, because drift is minimal | 50 samples (≈5 s) — short, track drift curve tightly |
| **Jump sensitivity** | High — GNSS jumps are authoritative, confirm quickly | Low — must distinguish from drift acceleration |
| **PPS** | Active — anchors absolute epoch | Disabled — no signal |
| **Absolute time** | Available via GNSS → PPS → UTC | Not available — all times relative to boot |
| **Failure mode** | GNSS loss → drift begins, PPS lost, warn user | Clock becomes uncalibrated, OLS tracks relative offset only |

### 4.3 Drone: aggressive drift compensation

Without GNSS, the drone's device oscillator drifts freely.  Temperature
sweep during flight (0°C at altitude → 40°C at ground) can shift the
crystal 10–20 ppm over minutes.  The OLS model handles this naturally
because drift appears as α ≠ 1.0 in the model.

**Key: the drift rate itself drifts.**  α is not constant — it changes
with temperature.  The short OLS window (50 samples ≈ 5 s) ensures
the model tracks the *current* drift, not a historical average.

### 4.4 Car: GNSS recovery handling

After exiting a tunnel, the GNSS receiver reacquires satellites.  The
device timestamp may jump by 1–500 ms as the receiver's internal clock
re-synchronises to GPS time.  Sequence of events:

```
1. Tunnel entry:   PPS lost (PpsLost event)
2. Tunnel transit: OLS model continues on free-running host clock.
                   Drift slowly accumulates (< 100 ns/s × seconds).
                   offset_stddev_ns increases over time.
3. Tunnel exit:    GNSS reacquires.
                   First PPS → offset jumps.
                   ClockService detects as TimeJump.
                   Model resets and re-calibrates within 3 samples.
4. Few seconds:    PPS re-locks.  offset_stddev_ns returns to normal.
```

---

## 5. Failure Handling Strategy

### 5.1 Failure modes and recovery

| Failure | Detection | Recovery | Event |
|---------|-----------|----------|-------|
| **No hardware timestamps** (hw_ns = 0) | `observe()` checks hw_ns | Fall back to host timestamps only; `calibrated` stays false | `ModelReset` |
| **Excessive drift** (> threshold) | OLS α far from 1.0 | Warn user, continue estimating (may indicate hardware fault) | `DriftWarning` |
| **Time jump** | N consecutive outliers | Reset model, re-seed, continue | `TimeJump` |
| **PPS loss** | No PPS pulse for N intervals | Fall back to OLS-only (no absolute anchor) | `PpsLost` |
| **PPS jitter** | |Δt − 1s| > pps_tolerance | Reject pulse, don't corrupt model | (silent reject) |
| **Transport reconnect** | `ConnectionManager` reconnect event | `reset()` the ClockService; re-calibrate from scratch | `ModelReset` |
| **Model divergence** | offset_stddev_ns > degraded threshold | Flag DiagLevel::Degraded so consumers can widen tolerance | via `diagnostics()` |
| **Zero observations** | No `observe()` calls for > 1 s | Not a clock failure — transport layer issue | (not our concern) |

### 5.2 Graceful degradation tiers

```
Tier 0 — LOCKED:
    PPS + OLS calibrated.  σ < 1 μs.
    All timestamps are absolute UTC-aligned.
    (Car with GNSS + SO_TIMESTAMPING)

Tier 1 — CALIBRATED:
    OLS calibrated, no PPS.  σ < 100 μs.
    Timestamps are relative (host-domain) but drift-compensated.
    (Drone with SO_TIMESTAMPING, or Car without PPS)

Tier 2 — ESTIMATED:
    OLS calibrated with software timestamps.  σ 100 μs – 1 ms.
    Timestamps are drift-compensated but with scheduling jitter.
    (Any platform without SO_TIMESTAMPING)

Tier 3 — RAW:
    OLS not yet calibrated (< 3 samples) or hardware timestamps = 0.
    Using raw host timestamps (Timestamp::now()).
    (Startup transient, or device doesn't provide HW timestamps)
```

Downstream consumers (sync engines, SLAM, perception) can query the
tier via `ClockService::diagnostics()` and adjust their behavior:
e.g., `TimeSyncEngine` could widen `camera_tolerance_ns` when at Tier 2+.

---

## 6. Integration with Existing Code

### 6.1 DeviceManager owns the ClockService

```cpp
// device_manager.cpp  (Impl struct)
struct Impl {
    ClockService clock_service;          // ← NEW
    SyncEngine sync_engine;
    data::TimeSyncEngine time_sync_engine;
    // SlamTimeSync is owned by OdometryManager, but receives
    // a ClockService* on construction.
    ...
};
```

### 6.2 PacketParser feeds observations

Currently in `PacketParser::dispatch()`:
```cpp
Timestamp host_ts = Timestamp::now();
Timestamp hw_ts{hdr.hw_timestamp_ns};
```

After integration:
```cpp
Timestamp host_ts = host_timestamp();  // SO_TIMESTAMPING or now()
Timestamp hw_ts{hdr.hw_timestamp_ns};

// Feed the clock service (if pointer set)
if (clock_service_) {
    clock_service_->observe(hw_ts.nanoseconds, host_ts.nanoseconds);
}
```

The parser doesn't *use* `ClockService` for its output timestamps — it
still emits raw (hw, host) pairs.  The sync engines downstream do the
conversion.

### 6.3 SlamTimeSync migration

Currently owns an internal `ClockDriftModel`.  Migration:

```cpp
// Before:
ClockDriftModel drift_model_;
int64_t resolve_timestamp(int64_t hw_ns) const {
    if (drift_model_.is_valid())
        return drift_model_.hw_to_host(hw_ns);
    return hw_ns;
}

// After:
ClockService* clock_service_{nullptr};  // injected
int64_t resolve_timestamp(int64_t hw_ns) const {
    if (clock_service_ && clock_service_->is_calibrated())
        return clock_service_->hw_to_host(hw_ns);
    return hw_ns;
}
```

`SlamTimeSync::feed_imu(sample, host_ns)` no longer calls
`drift_model_.add_observation()` — the parser already fed it to
`ClockService`.

### 6.4 TimeSyncEngine migration

Currently has its own drift regression (`update_drift()`).  Migration:

```cpp
// Remove: offset_history_, ts_history_, drift_ns_per_sec_
// Remove: update_drift() method
// Add:    ClockService* clock_service_{nullptr};

// In try_assemble(), replace:
//   update_drift(offset_ns, ref_ts);
// with:
//   stats_.drift_ns_per_sec = clock_service_
//       ? clock_service_->drift_ns_per_sec() : 0.0;
```

### 6.5 Observation decimation

IMU at 400 Hz provides 400 (hw, host) pairs per second.  The OLS model
doesn't benefit from >50–100 samples/second (diminishing returns).
The `ClockService` can internally decimate:

```cpp
void observe(int64_t hw_ns, int64_t host_ns) {
    ++total_observations_;
    // Decimate: accept every Nth observation (adaptive based on rate)
    if (total_observations_ % decimate_factor_ != 0) return;
    // ... proceed with OLS ...
}
```

Or the caller (parser) can decimate.  Either way, the per-packet
overhead is one branch.

---

## 7. Timestamp Flow — Before and After

### 7.1 Before (current state)

```
Device HW clock                            Host steady_clock
      │                                          │
      ▼                                          │
  PacketHeader::hw_timestamp_ns                  │
      │                                          │
      ▼                                          │
  PacketParser::dispatch()                       │
      │                                          │
      │  frame->timestamp = hw_ts    ◄─ from packet
      │  frame->host_timestamp       ◄─ Timestamp::now()  ──── Host
      │                                          │
      ├──► SyncEngine (uses hw only, no drift)   │
      ├──► TimeSyncEngine (drift estimated but   │
      │    never applied; only warns)            │
      └──► SlamTimeSync (ClockDriftModel, drift  │
           compensated in ESIKF path only)       │
```

### 7.2 After (proposed)

```
Device HW clock                            Host clock (steady / realtime / SO_TS)
      │                                          │
      ▼                                          ▼
  PacketHeader::hw_timestamp_ns           SO_TIMESTAMPING RX / now()
      │                                          │
      ▼                                          │
  PacketParser::dispatch()                       │
      │                                          │
      │  observe(hw_ns, host_ns) ──────►  ClockService
      │                                      │
      │  frame->timestamp = hw_ts            │ OLS model
      │  frame->host_timestamp = host_ts     │ α, β, σ
      │                                      │
      ├──► SyncEngine ──────────────────► clock_.unified_timestamp()
      │                                      │
      ├──► TimeSyncEngine ──────────────► clock_.unified_timestamp()
      │                                      │
      └──► SlamTimeSync ───────────────► clock_.hw_to_host()
                                             │
                                     (optional PPS anchor)
                                             │
                                        Unified time
```

---

## 8. File Changes Summary

| File | Change | New/Modify |
|------|--------|-----------|
| `sdk/include/thunderbird/clock_service.h` | `ClockService`, `ClockOffsetEstimator`, `ClockProfile`, `ClockServiceConfig`, `ClockDiagnostics` | New |
| `sdk/src/clock_service.cpp` | Implementation: OLS + outlier rejection + jump detection + seqlock + PPS | New |
| `sdk/include/thunderbird/odom/slam_time_sync.h` | Replace internal `ClockDriftModel` with `ClockService*` injection. Keep `ClockDriftModel` class in place for backward compat (used by nothing else, but harmless). | Modify |
| `sdk/include/thunderbird/time_sync.h` | Remove internal drift regression (`update_drift`, `offset_history_`, `ts_history_`). Accept `ClockService*`. | Modify |
| `sdk/include/thunderbird/sync_engine.h` | Accept `ClockService*`.  In `find_nearest()`, use `unified_timestamp()` for comparison. | Modify |
| `sdk/src/device_manager.cpp` | Own `ClockService`.  Pass pointer to all sync engines and parser. | Modify |
| `sdk/include/thunderbird/packet_parser.h` | Accept `ClockService*`, call `observe()` from `dispatch()`. | Modify |
| `sdk/include/thunderbird/types.h` | Add `HostClockSource` enum. | Modify |

### 8.1 Implementation order

1. **`clock_service.h/cpp`** — standalone, no dependencies on existing engines
2. **`DeviceManager` integration** — inject `ClockService*` into existing engines
3. **`PacketParser` observation injection** — call `observe()` from `dispatch()`
4. **`SlamTimeSync` migration** — replace `ClockDriftModel` usage with `ClockService*`
5. **`TimeSyncEngine` migration** — remove internal drift regression
6. **`SyncEngine` migration** — add `unified_timestamp()` calls
7. **PPS support** — add `observe_pps()` implementation and car-profile config
8. **Tests** — unit tests for jump detection, outlier rejection, drift
   estimation accuracy, seqlock correctness, PPS lock/loss scenarios

---

## 9. What This Design Does NOT Include

| Item | Reason |
|------|--------|
| PTP protocol implementation | OS/infrastructure concern, not SDK (§3.3) |
| GPS/NMEA parsing | Separate driver responsibility; we just consume PPS edges |
| Temperature-compensated oscillator modeling | Polynomial XO models are device-specific; linear OLS is sufficient for the SDK layer — device firmware should do TCXO compensation |
| Kalman filter for clock estimation | OLS is simpler, equally accurate for slowly-varying drift, and doesn't require process-noise tuning. If profiling shows OLS is insufficient, a KF can be swapped in behind the same `ClockService` interface |
| Multi-device clock synchronization | Current architecture is one ClockService per DeviceManager (one device). Multi-device sync is a higher-level concern |
