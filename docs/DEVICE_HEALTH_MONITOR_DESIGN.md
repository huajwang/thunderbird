# Device Health Monitor — Design Document

**Component**: `DeviceHealthMonitor`
**Header**: `sdk/include/thunderbird/device_health_monitor.h`
**Layer**: SDK — sits alongside `ConnectionManager`, consumed by `DeviceManager` and `SlamDaemon`
**Thread safety**: All public methods are thread-safe; monitor tick runs on a dedicated thread
**Status**: Design — Phase 2 hardware integration

---

## 0. Existing Infrastructure Audit

Before designing anything new, a full audit of the codebase was conducted.
The following subsystems already exist and work correctly for their intended scope:

### 0.1 ConnectionStateMachine (`connection.h`)

Six states: `Disconnected → Connecting → Handshake → Connected → Streaming → Error`.
Heartbeat monitoring on a background thread: sends periodic heartbeats, counts
missed replies, transitions to `Error` after `heartbeat_miss_limit` (default 3)
consecutive misses.  **Binary health model** — no intermediate degraded state.

### 0.2 ConnectionManager (`connection_manager.h`)

Wraps `ConnectionStateMachine` + `ITransport` + `PacketParser`.  Provides:

- **Exponential-backoff retry** with jitter on initial connect.
- **Auto-reconnect** on heartbeat timeout (`Error` state triggers `attempt_reconnect()`
  on a detached thread).
- **Single I/O thread** that reads from transport and feeds the parser.
- **`ConnectionEvent` callbacks**: Connected, Disconnected, Reconnecting,
  ReconnectFailed, StreamStarted, StreamStopped, HeartbeatTimeout, ProtocolError.
- `RetryConfig`: initial_retry_delay_ms=500, backoff_multiplier=2.0,
  max_retry_delay_ms=30000, max_connect_attempts=5, max_reconnect_attempts=10.

### 0.3 ParserStats (`packet_parser.h`)

```cpp
struct ParserStats {
    uint64_t packets_parsed{0};
    uint64_t crc_errors{0};
    uint64_t resync_count{0};
    uint64_t bytes_processed{0};
};
```

Counters are accumulated but **nobody monitors rates or rate changes**.

### 0.4 SlamHealthMonitor (`odom/slam_health.h`)

SLAM-algorithm-specific.  States: Nominal → Degraded → Critical → Emergency.
10 fault flags (ImuDropout, LidarDropout, EkfDivergence, etc.).  Monitors
algorithm outputs — covariance trace, residual spikes, drift rate, map size,
geometry degeneracy.  **Does NOT monitor the transport layer or device health.**

### 0.5 SlamDaemon healthLoop (`acme_slamd.cpp`)

System-level health thread.  Monitors CPU usage, RSS, tracking status, sensor
connected/streaming (simple booleans).  Has `drift_ns_per_sec = 0; // TODO: wire
to SlamTimeSync drift model`.  Publishes systemd status.  **No packet rate
monitoring, no transport stall detection, no degraded-state logic.**

### 0.6 HardwareDriver / DeviceManager

`HardwareDriver` is a thin wrapper around `ConnectionManager` — no health logic.
`DeviceManager` is the user-facing API — connect/disconnect/start/stop.

---

## 1. Problem Statement

The existing codebase has a **binary view of device health**: connected or broken.
On real hardware, degradation is gradual:

| Failure mode | Current detection | Consequence |
|---|---|---|
| **Packet rate drops 50%** (thermal throttle, EMI, loose connector) | None | SLAM degrades silently; user gets stale data without knowing |
| **Transport stall** (OS buffer full, driver freeze, cable half-unplugged) | Only via heartbeat timeout (3 s minimum lag) | 3 seconds of undetected data loss before hard disconnect |
| **Per-sensor dropout** (LiDAR stops but IMU continues) | `SlamHealthMonitor` detects post-hoc in algorithm | No transport-level awareness; reconnect not triggered |
| **CRC error spike** (interference, underpowered USB, bad cable) | Counter exists in `ParserStats` but nobody watches it | Silent data corruption rate increase |
| **Gradual reconnect loop** (flaky connection, marginal signal) | `ConnectionManager` reconnects but no flap detection | Reconnect storm consumed silently; no exponential cooldown |

The system needs a **device-level health monitor** that:

1. Tracks per-sensor packet rates over time.
2. Detects transport stall (zero data flow) faster than heartbeat timeout.
3. Provides a **DEGRADED** state between CONNECTED and DISCONNECTED.
4. Aggregates transport metrics + parser stats + heartbeat status.
5. Provides structured device health snapshots with hardware status.
6. Integrates with the existing `ConnectionManager` reconnect logic without replacing it.

---

## 2. Architecture

```
┌───────────────────────────────────────────────────────────┐
│ DeviceManager                                             │
│   ┌─────────────────────┐  ┌────────────────────────────┐ │
│   │ HardwareDriver      │  │ DeviceHealthMonitor  ★ NEW │ │
│   │   └─ ConnectionMgr  │──│   ├─ RateTracker (per-sensor)│
│   │        ├─ ITransport │  │   ├─ StallDetector         │ │
│   │        ├─ PacketParser──│   ├─ CrcMonitor            │ │
│   │        └─ StateMachine──│   ├─ FlapDetector          │ │
│   └─────────────────────┘  │   └─ HealthStateMachine    │ │
│                             └────────────────────────────┘ │
│                                        │                   │
│                              DeviceHealthSnapshot          │
│                              DeviceHealthState             │
│                              DeviceHealthCallback          │
└───────────────────────────────────────────────────────────┘
```

**Ownership**: `DeviceManager` owns both `HardwareDriver` and `DeviceHealthMonitor`.
The health monitor holds non-owning references to `ConnectionManager` (for state/events)
and `PacketParser` (for stats snapshots).

**Data flow**: The I/O thread (owned by `ConnectionManager`) already calls
`parser_.feed()`.  The health monitor does NOT intercept this hot path.  Instead, it
**polls `ParserStats` and `ConnectionState` on its own timer thread** (1–10 Hz,
configurable).  This keeps the sensor data path zero-overhead.

---

## 3. State Machine

Three states, matching the user's requirement.

```
                    ┌──── timeout ────┐
                    ▼                 │
  ┌─────────────┐  ┌─────────────┐  ┌──────────────┐
  │ DISCONNECTED│──▶│  CONNECTED  │──▶│   DEGRADED   │
  └──────▲──────┘  └─────────────┘  └──────────────┘
         │             ▲      │          │       │
         │             │      │          │       │
         │    reconnect│  any │   heal   │  stall│
         │    success  │ fault│  window  │   /   │
         │             │      ▼          │ total │
         │             └──────┘          │ loss  │
         │                               │       │
         └───────────────────────────────┘       │
         └───────────────────────────────────────┘
```

### 3.1 State Definitions

```cpp
enum class DeviceHealthState : uint8_t {
    Disconnected,   // transport is not open; no data expected
    Connected,      // all sensor streams active, rates within tolerance
    Degraded,       // one or more degradation signals detected
};
```

### 3.2 Transition Rules

| From | To | Trigger | Detail |
|---|---|---|---|
| Disconnected | Connected | `ConnectionEvent::StreamStarted` received AND first measurement arrives | Requires at least one packet from each enabled sensor |
| Connected | Degraded | **Any** degradation fault fires | See §4 fault detectors |
| Degraded | Connected | All faults clear for `heal_window` (default 5 s) | Hysteresis prevents flapping |
| Degraded | Disconnected | Transport stall > `stall_disconnect_threshold` (default 10 s) OR `ConnectionEvent::Disconnected` received | Hard cutover — reconnect logic takes over |
| Connected | Disconnected | `ConnectionEvent::Disconnected` or `HeartbeatTimeout` | Existing ConnectionManager handles reconnect |
| Disconnected | (no change) | `ConnectionEvent::Reconnecting` | Stay in Disconnected until streaming resumes |

### 3.3 State vs. `ConnectionManager` interaction

The `DeviceHealthMonitor` does **not** own or replace the reconnect logic.
When it transitions to `Disconnected`, it notifies listeners via callback.
`ConnectionManager` continues to handle the actual reconnection.  After successful
reconnect + streaming restart, `ConnectionManager` emits `StreamStarted` and the
health monitor returns to `Connected` (or `Degraded` if faults remain).

---

## 4. Fault Detectors

Five independent detectors, each with a boolean `fired` state and a score [0, 1].
Inspired by the `SlamHealthMonitor` detector pattern, but applied to
transport/device metrics instead of algorithm metrics.

### 4.1 RateDropDetector — per-sensor packet rate

```cpp
struct RateDropDetector {
    SensorType sensor;          // LiDAR, IMU, Camera
    double     expected_hz;     // from DeviceConfig (lidar_hz, imu_hz, camera_fps)
    double     warn_ratio;      // 0.7 — fire if measured < 70% of expected
    double     crit_ratio;      // 0.3 — score drops to 0 below this

    // --- Internal state ---
    uint64_t   last_count{0};   // previous-tick packet count
    double     measured_hz{0};
    double     score{1.0};
    bool       fired{false};
};
```

**Measurement**: On each monitor tick, read `ParserStats::packets_parsed` and compute
the delta since the previous tick.  The per-sensor breakdown is achieved by
**intercepting the dispatch callbacks** (not the raw bytes).  Specifically,
`DeviceHealthMonitor` registers lightweight counting callbacks on the `PacketParser`:

```cpp
// Registered once at construction — runs on the I/O thread, must be cheap.
parser.on_lidar([this](auto) { lidar_count_.fetch_add(1, relaxed); });
parser.on_imu([this](auto)   { imu_count_.fetch_add(1, relaxed);   });
parser.on_camera([this](auto) { camera_count_.fetch_add(1, relaxed); });
```

These atomics are read/reset on the monitor thread.  **Cost: one relaxed atomic increment
per packet on the I/O thread — 2 ns, cache-friendly.** No heap allocation, no lock.

**Score**:

```
ratio = measured_hz / expected_hz

score = 1.0                       if ratio >= 1.0
      = (ratio - crit) / (warn - crit)   if crit < ratio < warn
      = 0.0                       if ratio <= crit
```

**Fault fires** when `score < warn_ratio` for any enabled sensor.

### 4.2 StallDetector — transport data flow

```cpp
struct StallDetector {
    uint64_t  stall_threshold_ms;     // default 2000 (2 s)
    uint64_t  disconnect_threshold_ms; // default 10000 (10 s)

    // --- Internal ---
    uint64_t  last_bytes_processed{0};
    TimePoint last_data_seen;
    uint64_t  stall_duration_ms{0};
    bool      fired{false};
    double    score{1.0};
};
```

**Measurement**: Compare `ParserStats::bytes_processed` across ticks.  If unchanged
for `stall_threshold_ms`, fire.  If unchanged for `disconnect_threshold_ms`,
trigger transition to `Disconnected`.

This detects transport-level freeze **before** the heartbeat timeout (default 3 s),
because stall detection runs at the monitor tick rate (default 2 Hz → 500 ms granularity)
and uses a 2 s threshold, while heartbeat timeout requires 3 missed beats × 1 s interval.

**Score**: `1.0 - clamp(stall_duration_ms / disconnect_threshold_ms, 0, 1)`.

### 4.3 CrcErrorDetector — data integrity

```cpp
struct CrcErrorDetector {
    double   error_rate_warn;    // 0.01 — 1% packet loss
    double   error_rate_crit;    // 0.05 — 5% packet loss (fire)

    // --- Internal ---
    uint64_t prev_crc_errors{0};
    uint64_t prev_packets{0};
    double   error_rate{0};
    bool     fired{false};
    double   score{1.0};
};
```

**Measurement**: On each tick, compute
`error_rate = Δcrc_errors / (Δpackets_parsed + Δcrc_errors)`.
A sustained high error rate indicates cable/EMI/power issues.

### 4.4 FlapDetector — reconnect storm

```cpp
struct FlapDetector {
    uint32_t  max_reconnects_per_window; // default 5
    double    window_s;                  // default 60.0

    // --- Internal ---
    std::array<TimePoint, 16> reconnect_times;  // ring buffer, fixed size
    size_t    ring_head{0};
    uint32_t  reconnects_in_window{0};
    bool      fired{false};
    double    score{1.0};
};
```

**Measurement**: Subscribe to `ConnectionEvent::Reconnecting` from `ConnectionManager`.
Record timestamps in a fixed-size ring buffer.  If `max_reconnects_per_window` is
exceeded, fire.  This detects the "marginal connection" scenario where heartbeat
barely passes but the link keeps dropping.

### 4.5 HeartbeatJitterDetector — heartbeat quality

```cpp
struct HeartbeatJitterDetector {
    double   max_jitter_ratio;   // 2.0 — fire if RTT > 2× expected interval

    // --- Internal ---
    int64_t  last_heartbeat_rtt_us{0};
    double   ema_rtt_us{0};
    bool     fired{false};
    double   score{1.0};
};
```

**Measurement**: When a heartbeat ack is received, compute round-trip time.
Track an exponential moving average.  If the RTT spikes above `max_jitter_ratio ×
heartbeat_interval_ms`, fire.  This provides early warning of network congestion
or CPU starvation before actual heartbeat timeout.

**Note**: This requires a minor extension to `ConnectionStateMachine` — expose the
heartbeat RTT alongside the existing `notify_heartbeat_received()` method.

---

## 5. Configuration

```cpp
struct DeviceHealthConfig {
    /// Monitor tick rate (Hz).  Higher = more responsive but more CPU.
    double      tick_hz                       = 2.0;

    /// Time with no faults before returning to Connected (ms).
    uint32_t    heal_window_ms                = 5000;

    // ── Rate drop ───────────────────────────────────────────────────────
    /// Fraction of expected rate below which the rate drop detector fires.
    double      rate_warn_ratio               = 0.70;
    /// Fraction of expected rate at which the score drops to 0.
    double      rate_crit_ratio               = 0.30;

    // ── Stall ───────────────────────────────────────────────────────────
    /// How long zero data flow triggers DEGRADED (ms).
    uint32_t    stall_threshold_ms            = 2000;
    /// How long zero data flow triggers DISCONNECTED (ms).
    uint32_t    stall_disconnect_threshold_ms = 10000;

    // ── CRC ─────────────────────────────────────────────────────────────
    /// Per-tick error rate above which the CRC detector fires.
    double      crc_error_rate_warn           = 0.01;
    double      crc_error_rate_crit           = 0.05;

    // ── Flap ────────────────────────────────────────────────────────────
    /// Max reconnects within `flap_window_s` before flap detector fires.
    uint32_t    max_reconnects_per_window     = 5;
    double      flap_window_s                 = 60.0;

    // ── Heartbeat jitter ────────────────────────────────────────────────
    double      heartbeat_jitter_ratio        = 2.0;

    // ── Per-sensor expected rates (overridden from DeviceConfig) ────────
    double      expected_lidar_hz             = 10.0;
    double      expected_imu_hz               = 200.0;
    double      expected_camera_fps           = 30.0;

    // ── Enabled sensors (bitmask, so we don't false-alarm on unused) ───
    bool        lidar_enabled                 = true;
    bool        imu_enabled                   = true;
    bool        camera_enabled                = true;
};
```

---

## 6. Snapshot & Callbacks

### 6.1 DeviceHealthSnapshot

```cpp
struct DeviceHealthSnapshot {
    // ── Top-level state ─────────────────────────────────────────────────
    DeviceHealthState  state{DeviceHealthState::Disconnected};
    int64_t            timestamp_ns{0};  // steady_clock at snapshot time
    int64_t            state_entered_ns{0}; // when current state began

    // ── Per-sensor rate info ────────────────────────────────────────────
    double  lidar_hz{0};
    double  imu_hz{0};
    double  camera_fps{0};
    double  lidar_rate_score{1.0};
    double  imu_rate_score{1.0};
    double  camera_rate_score{1.0};

    // ── Transport metrics ───────────────────────────────────────────────
    uint64_t bytes_total{0};
    uint64_t packets_total{0};
    uint64_t crc_errors_total{0};
    double   crc_error_rate{0};           // current tick's error rate
    uint64_t stall_duration_ms{0};
    double   stall_score{1.0};

    // ── Reconnect info ──────────────────────────────────────────────────
    uint32_t reconnect_count{0};          // total since monitor started
    uint32_t reconnects_in_window{0};     // in current flap window
    bool     flap_detected{false};

    // ── Heartbeat ───────────────────────────────────────────────────────
    double   heartbeat_rtt_ms{0};         // latest RTT (0 if unavailable)
    double   heartbeat_rtt_ema_ms{0};     // smoothed RTT
    bool     heartbeat_jitter{false};

    // ── Connection state (passthrough from ConnectionManager) ──────────
    ConnectionState connection_state{ConnectionState::Disconnected};

    // ── Aggregate health score ──────────────────────────────────────────
    double   health_score{1.0};           // weighted combination of all scores

    // ── Active faults (bitfield, same pattern as SlamHealthMonitor) ────
    uint32_t active_faults{0};

    // ── Device info (static, from handshake) ────────────────────────────
    DeviceInfo device_info;
};
```

### 6.2 Fault flags

```cpp
enum class DeviceFault : uint32_t {
    None              = 0,
    LidarRateDrop     = 1 << 0,
    ImuRateDrop       = 1 << 1,
    CameraRateDrop    = 1 << 2,
    TransportStall    = 1 << 3,
    CrcErrorSpike     = 1 << 4,
    ReconnectFlap     = 1 << 5,
    HeartbeatJitter   = 1 << 6,
};
```

Bitwise operators provided (same pattern as `FaultFlags` in `slam_health.h`).

### 6.3 Callbacks

```cpp
/// Fired on every monitor tick with the latest snapshot.
using DeviceHealthUpdateCallback =
    std::function<void(const DeviceHealthSnapshot&)>;

/// Fired only on state transitions.
using DeviceHealthStateCallback =
    std::function<void(DeviceHealthState from, DeviceHealthState to,
                       uint32_t active_faults)>;

/// Fired when a fault is first detected (rising edge).
using DeviceFaultCallback =
    std::function<void(DeviceFault fault, const DeviceHealthSnapshot&)>;
```

---

## 7. Threading Model

```
┌────────────────────────────┐
│  I/O Thread                │   (owned by ConnectionManager)
│  ┌──────────────────────┐  │
│  │ transport→read()     │  │
│  │ parser.feed()        │──┼──▶ atomic count increments (2 ns)
│  │                      │  │    lidar_count_, imu_count_, camera_count_
│  └──────────────────────┘  │
└────────────────────────────┘

┌────────────────────────────┐
│  Heartbeat Thread          │   (owned by ConnectionStateMachine)
│  sends heartbeat           │
│  checks missed count       │──▶ existing, no changes
└────────────────────────────┘

┌────────────────────────────┐
│  Health Monitor Thread ★   │   (owned by DeviceHealthMonitor)
│  ┌──────────────────────┐  │
│  │ tick():              │  │
│  │   read atomic counts │  │   load + reset (relaxed, 3 ns)
│  │   read ParserStats   │  │   snapshot copy (10 ns)
│  │   read ConnState     │  │   atomic load (1 ns)
│  │   run 5 detectors    │  │   ~50 ns total
│  │   update state machine│ │   branch + compare
│  │   fire callbacks     │  │   user-controlled cost
│  │   sleep(tick_period) │  │
│  └──────────────────────┘  │
└────────────────────────────┘
```

### 7.1 Thread interactions

| Interaction | Mechanism | Cost |
|---|---|---|
| I/O → Health (packet counts) | `std::atomic<uint64_t>` with `relaxed` ordering | 2 ns per packet |
| I/O → Health (ParserStats) | Health thread reads stats snapshot; I/O thread updates during `feed()` | No lock — counters are monotonic and worst-case tearing of a single uint64 on 32-bit is benign |
| ConnectionMgr → Health (events) | Monitor registers a `ConnectionEventCallback` | Callback runs on the state-change source thread; monitor enqueues event into a lock-free SPSC queue for processing on its own thread |
| Health → User (callbacks) | User-registered callbacks are invoked on the health monitor thread | User must not block |

### 7.2 Startup / shutdown ordering

```
start:
  1. ConnectionManager.connect()       → transport opens, handshake
  2. ConnectionManager.start_streaming() → I/O thread starts
  3. DeviceHealthMonitor.start()        → monitor thread starts, begins ticking

stop:
  1. DeviceHealthMonitor.stop()         → monitor thread joins
  2. ConnectionManager.stop_streaming()  → I/O thread stops
  3. ConnectionManager.disconnect()      → transport closes
```

---

## 8. Class Interface

```cpp
class DeviceHealthMonitor {
public:
    /// Construct with references to existing infrastructure.
    /// Does NOT take ownership of conn_mgr or parser.
    explicit DeviceHealthMonitor(ConnectionManager& conn_mgr,
                                  PacketParser& parser,
                                  DeviceHealthConfig config = {});
    ~DeviceHealthMonitor();

    // Non-copyable
    DeviceHealthMonitor(const DeviceHealthMonitor&) = delete;
    DeviceHealthMonitor& operator=(const DeviceHealthMonitor&) = delete;

    // ── Lifecycle ───────────────────────────────────────────────────────

    /// Start the monitor thread.  Call after streaming begins.
    void start();

    /// Stop the monitor thread.  Blocks until joined.
    void stop();

    bool is_running() const;

    // ── State queries (thread-safe) ─────────────────────────────────────

    DeviceHealthState state() const;
    DeviceHealthSnapshot snapshot() const;
    double health_score() const;
    uint32_t active_faults() const;

    // ── Callbacks ───────────────────────────────────────────────────────

    void on_health_update(DeviceHealthUpdateCallback cb);
    void on_state_change(DeviceHealthStateCallback cb);
    void on_fault(DeviceFaultCallback cb);

    // ── Manual overrides ────────────────────────────────────────────────

    /// Force a state transition (e.g. external watchdog wants to force disconnect).
    void force_state(DeviceHealthState s);

    /// Reset all detector state (e.g. after successful reconnect).
    void reset();

    // ── External event injection ────────────────────────────────────────

    /// Notify monitor of a heartbeat RTT measurement (called from parser
    /// control callback when HeartbeatAck is received).
    void notify_heartbeat_rtt(int64_t rtt_us);

    /// Notify monitor that a connection event occurred.
    /// Normally auto-wired via ConnectionManager callback, but exposed
    /// for testing.
    void notify_connection_event(ConnectionEvent event);

private:
    void monitor_loop();         // main tick loop
    void tick();                 // single monitor iteration
    void update_rate_detectors();
    void update_stall_detector();
    void update_crc_detector();
    void update_flap_detector();
    void update_heartbeat_detector();
    void evaluate_state_machine();
    void emit_callbacks();

    // --- References (non-owning) ---
    ConnectionManager& conn_mgr_;
    PacketParser&      parser_;
    DeviceHealthConfig cfg_;

    // --- Per-sensor atomic counters (incremented on I/O thread) ---
    std::atomic<uint64_t> lidar_count_{0};
    std::atomic<uint64_t> imu_count_{0};
    std::atomic<uint64_t> camera_count_{0};

    // --- Detectors ---
    RateDropDetector    rate_lidar_;
    RateDropDetector    rate_imu_;
    RateDropDetector    rate_camera_;
    StallDetector       stall_;
    CrcErrorDetector    crc_;
    FlapDetector        flap_;
    HeartbeatJitterDetector hb_jitter_;

    // --- State machine ---
    std::atomic<DeviceHealthState> state_{DeviceHealthState::Disconnected};
    int64_t state_entered_ns_{0};
    int64_t last_fault_cleared_ns_{0};

    // --- Monitor thread ---
    std::atomic<bool> running_{false};
    std::thread       monitor_thread_;

    // --- Snapshot (protected by mutex for multi-field consistency) ---
    mutable std::mutex snapshot_mu_;
    DeviceHealthSnapshot latest_snapshot_;

    // --- Callbacks (protected by mutex) ---
    std::mutex cb_mu_;
    DeviceHealthUpdateCallback update_cb_;
    DeviceHealthStateCallback  state_cb_;
    DeviceFaultCallback        fault_cb_;
};
```

---

## 9. Integration Points

### 9.1 Into DeviceManager

```cpp
// device_manager.cpp — hardware path (non-simulated)

// After creating HardwareDriver and ConnectionManager:
auto health_cfg = DeviceHealthConfig{};
health_cfg.expected_lidar_hz  = config.lidar_hz;
health_cfg.expected_imu_hz    = config.imu_hz;
health_cfg.expected_camera_fps = config.camera_fps;

impl_->health_monitor = std::make_unique<DeviceHealthMonitor>(
    impl_->hw_driver->connection(),        // ConnectionManager&
    impl_->hw_driver->connection().parser(), // PacketParser&
    health_cfg);

// In start():
impl_->health_monitor->start();

// In stop():
impl_->health_monitor->stop();
```

Expose via public API:

```cpp
class DeviceManager {
public:
    // ... existing API ...

    /// Access the device health monitor.  Returns nullptr in simulated mode.
    DeviceHealthMonitor* health_monitor();
    const DeviceHealthMonitor* health_monitor() const;

    /// Convenience: current device health state.
    DeviceHealthState device_health() const;
};
```

### 9.2 Into SlamDaemon healthLoop

```cpp
// acme_slamd.cpp — healthLoop()

// Replace the simple boolean check:
//   hs.sensor_connected = device->is_connected();
//   hs.sensor_streaming = device->is_streaming();
// With structured health snapshot:

if (device_health_monitor) {
    auto dhs = device_health_monitor->snapshot();
    hs.sensor_connected    = (dhs.state != DeviceHealthState::Disconnected);
    hs.sensor_streaming    = (dhs.connection_state == ConnectionState::Streaming);
    hs.sensor_health_state = dhs.state;
    hs.lidar_hz            = dhs.lidar_hz;
    hs.imu_hz              = dhs.imu_hz;
    hs.device_health_score = dhs.health_score;

    // Log degraded state.
    if (dhs.state == DeviceHealthState::Degraded) {
        log.log(LogLevel::Warning, "health",
                "Device degraded: score=" + std::to_string(dhs.health_score) +
                " lidar=" + std::to_string(dhs.lidar_hz) + " Hz" +
                " imu=" + std::to_string(dhs.imu_hz) + " Hz");
    }
}
```

### 9.3 ConnectionManager extension — heartbeat RTT

One small addition to `ConnectionStateMachine`:

```cpp
// connection.h — heartbeat_loop(), after receiving ack:

void notify_heartbeat_received(int64_t rtt_us) {
    missed_heartbeats_.store(0, std::memory_order_release);
    last_heartbeat_rtt_us_.store(rtt_us, std::memory_order_release);
}

std::atomic<int64_t> last_heartbeat_rtt_us_{0};
```

The `DeviceHealthMonitor` reads this atomic on each tick.

### 9.4 Callback wiring from ConnectionManager

```cpp
// During DeviceHealthMonitor construction:

conn_mgr_.on_event([this](ConnectionEvent e, const std::string&) {
    notify_connection_event(e);
});
```

This reuses the existing `ConnectionEventCallback` mechanism — the health monitor
becomes another observer, not a replacement.

---

## 10. Health Score Computation

Weighted average of all detector scores (same approach as `SlamHealthMonitor`):

```
health_score = w_lidar  × lidar_rate_score
             + w_imu    × imu_rate_score
             + w_camera  × camera_rate_score
             + w_stall   × stall_score
             + w_crc     × crc_score
             + w_flap    × flap_score
             + w_hb      × heartbeat_score
```

Default weights:

| Detector | Weight | Rationale |
|---|---|---|
| LiDAR rate | 0.25 | Primary sensor for SLAM |
| IMU rate | 0.25 | Critical for state estimation |
| Camera rate | 0.10 | Optional sensor |
| Stall | 0.15 | Transport-level freeze |
| CRC errors | 0.10 | Data integrity |
| Flap | 0.10 | Connection stability |
| Heartbeat jitter | 0.05 | Early warning |

Weights are normalised by enabled sensors (if camera is disabled, its weight is
redistributed proportionally).

---

## 11. Tick Implementation Sketch

```cpp
void DeviceHealthMonitor::tick() {
    const auto now = steady_clock::now();
    const int64_t now_ns = duration_cast<nanoseconds>(
        now.time_since_epoch()).count();

    // ── 1. Sample atomic counters (reset-on-read) ───────────────────────
    const uint64_t lidar_delta  = lidar_count_.exchange(0, relaxed);
    const uint64_t imu_delta    = imu_count_.exchange(0, relaxed);
    const uint64_t camera_delta = camera_count_.exchange(0, relaxed);

    // ── 2. Sample ParserStats ───────────────────────────────────────────
    const ParserStats stats = parser_.stats();

    // ── 3. Compute elapsed time since last tick ─────────────────────────
    const double dt_s = duration<double>(now - last_tick_).count();
    last_tick_ = now;
    if (dt_s <= 0) return;

    // ── 4. Per-sensor rate computation ──────────────────────────────────
    const double lidar_hz  = static_cast<double>(lidar_delta)  / dt_s;
    const double imu_hz    = static_cast<double>(imu_delta)    / dt_s;
    const double camera_fps = static_cast<double>(camera_delta) / dt_s;

    // ── 5. Run detectors ────────────────────────────────────────────────
    update_rate_detector(rate_lidar_,  lidar_hz,  cfg_.expected_lidar_hz);
    update_rate_detector(rate_imu_,    imu_hz,    cfg_.expected_imu_hz);
    update_rate_detector(rate_camera_, camera_fps, cfg_.expected_camera_fps);
    update_stall_detector(stats, now_ns);
    update_crc_detector(stats);
    update_flap_detector(now_ns);
    update_heartbeat_detector();

    // ── 6. Aggregate faults ─────────────────────────────────────────────
    uint32_t faults = 0;
    if (cfg_.lidar_enabled  && rate_lidar_.fired)  faults |= u32(DeviceFault::LidarRateDrop);
    if (cfg_.imu_enabled    && rate_imu_.fired)    faults |= u32(DeviceFault::ImuRateDrop);
    if (cfg_.camera_enabled && rate_camera_.fired) faults |= u32(DeviceFault::CameraRateDrop);
    if (stall_.fired)     faults |= u32(DeviceFault::TransportStall);
    if (crc_.fired)       faults |= u32(DeviceFault::CrcErrorSpike);
    if (flap_.fired)      faults |= u32(DeviceFault::ReconnectFlap);
    if (hb_jitter_.fired) faults |= u32(DeviceFault::HeartbeatJitter);

    // ── 7. State machine ────────────────────────────────────────────────
    evaluate_state_machine(faults, now_ns);

    // ── 8. Build snapshot ───────────────────────────────────────────────
    DeviceHealthSnapshot snap;
    snap.state            = state_.load(acquire);
    snap.timestamp_ns     = now_ns;
    snap.state_entered_ns = state_entered_ns_;
    snap.lidar_hz         = lidar_hz;
    snap.imu_hz           = imu_hz;
    snap.camera_fps       = camera_fps;
    snap.lidar_rate_score = rate_lidar_.score;
    snap.imu_rate_score   = rate_imu_.score;
    snap.camera_rate_score = rate_camera_.score;
    snap.bytes_total      = stats.bytes_processed;
    snap.packets_total    = stats.packets_parsed;
    snap.crc_errors_total = stats.crc_errors;
    snap.crc_error_rate   = crc_.error_rate;
    snap.stall_duration_ms = stall_.stall_duration_ms;
    snap.stall_score      = stall_.score;
    snap.reconnect_count  = flap_.total_reconnects;
    snap.reconnects_in_window = flap_.reconnects_in_window;
    snap.flap_detected    = flap_.fired;
    snap.heartbeat_rtt_ms = hb_jitter_.last_rtt_ms;
    snap.heartbeat_rtt_ema_ms = hb_jitter_.ema_rtt_ms;
    snap.heartbeat_jitter = hb_jitter_.fired;
    snap.connection_state = conn_mgr_.state();
    snap.health_score     = compute_health_score();
    snap.active_faults    = faults;
    snap.device_info      = conn_mgr_.device_info();

    {
        std::lock_guard lk(snapshot_mu_);
        latest_snapshot_ = snap;
    }

    // ── 9. Callbacks ────────────────────────────────────────────────────
    emit_callbacks(snap, faults);
}
```

**Total estimated cost per tick**: ~100–200 ns of computation + callback overhead.
At 2 Hz, this is invisible.

---

## 12. Failure Scenarios & Recovery

| Scenario | Detection | State transition | Recovery action |
|---|---|---|---|
| **Thermal throttle**: LiDAR rate drops from 10 Hz to 6 Hz | RateDropDetector fires (60% < 70% warn) | Connected → Degraded | Log warning.  SLAM should widen tolerance. Auto-recover when rate returns. |
| **USB cable half-unplugged**: IMU stops, LiDAR continues | ImuRateDrop fires (0 Hz / 200 Hz = 0) | Connected → Degraded | After 10 s of stall on IMU, ConnectionManager has likely already reconnected via heartbeat.  If heartbeat still alive, stays Degraded until IMU resumes. |
| **Network outage**: All data stops, heartbeats fail | StallDetector fires at 2 s, HeartbeatTimeout at 3 s | Connected → Degraded → Disconnected | ConnectionManager auto-reconnect kicks in. DeviceHealthMonitor transitions to Connected on successful reconnect + first packets. |
| **Packet corruption (bad cable/EMI)**: 3% CRC error rate | CrcErrorDetector fires (3% > 1% warn) | Connected → Degraded | Log warning with error rate.  If > 5%, score = 0 → SLAM may pause intake. Self-recovers when cable is fixed. |
| **Flaky WiFi**: connects, streams for 10 s, drops, reconnects, loops | FlapDetector fires (>5 reconnects/minute) | Degraded remains (never stably returns to Connected) | Log "reconnect flap detected".  May escalate to Disconnected if ConnectionManager exhausts max_reconnect_attempts. |
| **CPU starvation**: Monitor tick delayed, heartbeat replies delayed | HeartbeatJitterDetector fires (RTT > 2× interval) | Connected → Degraded | Log warning.  Application should reduce CPU-intensive tasks. |
| **Graceful recovery**: All issues resolve | All detectors clear for heal_window (5 s) | Degraded → Connected | State callback notifies application.  SLAM restores normal tolerances. |

---

## 13. Testing Strategy

### 13.1 Unit tests (no real hardware)

```
test_initial_state_disconnected
test_transition_to_connected_on_first_packets
test_lidar_rate_drop_triggers_degraded
test_imu_rate_drop_triggers_degraded
test_camera_rate_drop_triggers_degraded
test_stall_triggers_degraded_then_disconnected
test_crc_error_spike_triggers_degraded
test_reconnect_flap_detection
test_heartbeat_jitter_detection
test_heal_window_transitions_back_to_connected
test_multiple_simultaneous_faults
test_health_score_computation
test_score_weight_redistribution_disabled_sensor
test_snapshot_completeness
test_state_change_callback_fires_once_per_transition
test_fault_callback_fires_on_rising_edge_only
test_reset_clears_all_state
test_force_state_override
test_connection_event_forwarding
```

### 13.2 Integration tests (simulated transport)

```
test_end_to_end_with_simulated_transport_healthy
test_simulated_transport_stall_then_recovery
test_simulated_rate_drop_then_recovery
test_reconnect_cycle_with_health_monitor
```

### 13.3 Performance tests

```
test_tick_cost_under_100ns          // no fault case
test_atomic_counter_overhead        // I/O thread impact
test_snapshot_copy_under_1us        // mutex-protected copy
```

---

## 14. Why NOT Modify Existing Components

| Alternative considered | Reason rejected |
|---|---|
| Add DEGRADED to `ConnectionStateMachine` | The connection state machine models the **protocol** lifecycle (handshake, streaming, error). Packet rate degradation is not a protocol state — the connection is still open and streaming. Mixing transport-level and data-quality concerns in one state machine creates ambiguity: what does "Degraded" mean for `request_start_stream()`? |
| Add rate monitoring to `ConnectionManager::io_loop()` | Violates separation of concerns — the I/O loop should do exactly one thing: read bytes and feed the parser.  Adding health checks there creates coupling and potential performance issues on the hot path. |
| Extend `SlamHealthMonitor` | It's SLAM-algorithm-specific (covariance, geometry, residuals). Device transport health is orthogonal.  The two monitors complement each other: SlamHealthMonitor says "the math is wrong", DeviceHealthMonitor says "the data isn't arriving". |
| Build into `SlamDaemon::healthLoop()` | The daemon health loop is system-level (CPU, RSS, watchdog).  Device health monitoring belongs in the SDK so it's available to any application, not just the SLAM daemon. |

---

## 15. File Plan

```
sdk/include/thunderbird/device_health_monitor.h   — public header (enums, config, snapshot, class)
sdk/src/device_health_monitor.cpp                  — implementation
tests/test_device_health_monitor.cpp               — unit tests
```

No new dependencies.  Uses only existing SDK headers (`connection_manager.h`,
`packet_parser.h`, `types.h`) plus `<atomic>`, `<thread>`, `<mutex>`,
`<chrono>`, `<functional>`, `<array>`.

---

## 16. Open Questions

1. **Should DeviceHealthMonitor trigger reconnect directly?**  Current design: no —
   it only reports state.  `ConnectionManager` handles reconnection independently.
   If we wanted the health monitor to trigger a "force reconnect" (e.g. after
   sustained degradation), it would need a `ConnectionManager::force_reconnect()`
   method.  This is deferred to implementation.

2. **Should the DEGRADED state have sub-levels?**  (e.g. DEGRADED_WARN vs
   DEGRADED_CRITICAL).  Current design uses a single DEGRADED state plus
   the `health_score` float for granularity.  Applications that need
   finer-grained response should threshold on the score.

3. **Should the health monitor participate in systemd watchdog?**  Current design:
   no — the SlamDaemon healthLoop already handles `sd_notify(WATCHDOG=1)`.
   The health monitor provides data to that loop via snapshots.

4. **Heartbeat RTT measurement** requires modifying `ConnectionStateMachine` to
   record send-time and compute RTT on ack receipt.  This is a 5-line change
   but touches an existing class.  Acceptable?
