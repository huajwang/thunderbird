// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Device health monitor
// ─────────────────────────────────────────────────────────────────────────────
//
// Monitors transport-level / device-level health, complementing the
// algorithm-level SlamHealthMonitor.  Provides a three-state health model:
//
//   DISCONNECTED ──▶ CONNECTED ──▶ DEGRADED ──▶ DISCONNECTED
//                       ▲              │
//                       └── heal ──────┘
//
// Five independent fault detectors:
//   1. RateDropDetector   — per-sensor packet rate monitoring
//   2. StallDetector      — transport data-flow watchdog
//   3. CrcErrorDetector   — data integrity rate tracking
//   4. FlapDetector       — reconnect storm detection
//   5. HeartbeatJitterDetector — heartbeat RTT quality
//
// Design:
//   • Polling architecture — monitor thread polls ParserStats + atomic
//     per-sensor counters at a configurable tick rate (default 2 Hz).
//   • Hot-path overhead: one relaxed atomic increment per packet (~2 ns).
//   • All public methods are thread-safe.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/connection_manager.h"
#include "thunderbird/packet_parser.h"
#include "thunderbird/types.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>

namespace thunderbird {

// ─── Health states ──────────────────────────────────────────────────────────

enum class DeviceHealthState : uint8_t {
    Disconnected,   // transport is not open; no data expected
    Connected,      // all sensor streams active, rates within tolerance
    Degraded,       // one or more degradation signals detected
};

inline constexpr const char* device_health_state_name(DeviceHealthState s) {
    switch (s) {
        case DeviceHealthState::Disconnected: return "Disconnected";
        case DeviceHealthState::Connected:    return "Connected";
        case DeviceHealthState::Degraded:     return "Degraded";
    }
    return "Unknown";
}

// ─── Fault flags ────────────────────────────────────────────────────────────

enum class DeviceFault : uint32_t {
    None              = 0,
    LidarRateDrop     = 1u << 0,
    ImuRateDrop       = 1u << 1,
    CameraRateDrop    = 1u << 2,
    TransportStall    = 1u << 3,
    CrcErrorSpike     = 1u << 4,
    ReconnectFlap     = 1u << 5,
    HeartbeatJitter   = 1u << 6,
};

inline constexpr DeviceFault operator|(DeviceFault a, DeviceFault b) {
    return static_cast<DeviceFault>(static_cast<uint32_t>(a) |
                                    static_cast<uint32_t>(b));
}
inline constexpr DeviceFault operator&(DeviceFault a, DeviceFault b) {
    return static_cast<DeviceFault>(static_cast<uint32_t>(a) &
                                    static_cast<uint32_t>(b));
}
inline DeviceFault& operator|=(DeviceFault& a, DeviceFault b) {
    a = a | b; return a;
}
inline constexpr bool has_device_fault(DeviceFault flags, DeviceFault test) {
    return (static_cast<uint32_t>(flags) & static_cast<uint32_t>(test)) != 0;
}
inline constexpr uint32_t device_fault_count(DeviceFault f) {
    uint32_t v = static_cast<uint32_t>(f);
    uint32_t c = 0;
    while (v) { c += v & 1; v >>= 1; }
    return c;
}

inline const char* device_fault_name(DeviceFault f) {
    switch (f) {
        case DeviceFault::None:            return "None";
        case DeviceFault::LidarRateDrop:   return "LidarRateDrop";
        case DeviceFault::ImuRateDrop:     return "ImuRateDrop";
        case DeviceFault::CameraRateDrop:  return "CameraRateDrop";
        case DeviceFault::TransportStall:  return "TransportStall";
        case DeviceFault::CrcErrorSpike:   return "CrcErrorSpike";
        case DeviceFault::ReconnectFlap:   return "ReconnectFlap";
        case DeviceFault::HeartbeatJitter: return "HeartbeatJitter";
    }
    return "Unknown";
}

// ─── Configuration ──────────────────────────────────────────────────────────

struct DeviceHealthConfig {
    /// Monitor tick rate (Hz).  Higher = more responsive but more CPU.
    double      tick_hz                       = 2.0;

    /// Time with no faults before returning to Connected (ms).
    uint32_t    heal_window_ms                = 5000;

    // ── Rate drop ───────────────────────────────────────────────────────
    double      rate_warn_ratio               = 0.70;
    double      rate_crit_ratio               = 0.30;

    // ── Stall ───────────────────────────────────────────────────────────
    uint32_t    stall_threshold_ms            = 2000;
    uint32_t    stall_disconnect_threshold_ms = 10000;

    // ── CRC ─────────────────────────────────────────────────────────────
    double      crc_error_rate_warn           = 0.01;
    double      crc_error_rate_crit           = 0.05;

    // ── Flap ────────────────────────────────────────────────────────────
    uint32_t    max_reconnects_per_window     = 5;
    double      flap_window_s                 = 60.0;

    // ── Heartbeat jitter ────────────────────────────────────────────────
    double      heartbeat_jitter_ratio        = 2.0;

    // ── Per-sensor expected rates ───────────────────────────────────────
    double      expected_lidar_hz             = 10.0;
    double      expected_imu_hz               = 200.0;
    double      expected_camera_fps           = 30.0;

    // ── Enabled sensors ─────────────────────────────────────────────────
    bool        lidar_enabled                 = true;
    bool        imu_enabled                   = true;
    bool        camera_enabled                = true;
};

// ─── Snapshot ───────────────────────────────────────────────────────────────

struct DeviceHealthSnapshot {
    // ── Top-level state ─────────────────────────────────────────────────
    DeviceHealthState  state{DeviceHealthState::Disconnected};
    int64_t            timestamp_ns{0};
    int64_t            state_entered_ns{0};

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
    double   crc_error_rate{0};
    uint64_t stall_duration_ms{0};
    double   stall_score{1.0};

    // ── Reconnect info ──────────────────────────────────────────────────
    uint32_t reconnect_count{0};
    uint32_t reconnects_in_window{0};
    bool     flap_detected{false};

    // ── Heartbeat ───────────────────────────────────────────────────────
    double   heartbeat_rtt_ms{0};
    double   heartbeat_rtt_ema_ms{0};
    bool     heartbeat_jitter{false};

    // ── Connection state (passthrough) ──────────────────────────────────
    ConnectionState connection_state{ConnectionState::Disconnected};

    // ── Aggregate health score ──────────────────────────────────────────
    double   health_score{1.0};

    // ── Active faults ───────────────────────────────────────────────────
    DeviceFault active_faults{DeviceFault::None};

    // ── Device info ─────────────────────────────────────────────────────
    DeviceInfo device_info;
};

// ─── Callbacks ──────────────────────────────────────────────────────────────

using DeviceHealthUpdateCallback =
    std::function<void(const DeviceHealthSnapshot&)>;

using DeviceHealthStateCallback =
    std::function<void(DeviceHealthState from, DeviceHealthState to,
                       DeviceFault active_faults)>;

using DeviceFaultCallback =
    std::function<void(DeviceFault fault, const DeviceHealthSnapshot&)>;

// ─── Internal detector structs ──────────────────────────────────────────────

namespace detail {

struct RateDropDetector {
    double     expected_hz{0};
    double     warn_ratio{0.70};
    double     crit_ratio{0.30};

    uint64_t   last_count{0};
    double     measured_hz{0};
    double     score{1.0};
    bool       fired{false};

    void update(uint64_t count_delta, double dt_s) {
        if (dt_s <= 0) return;
        measured_hz = static_cast<double>(count_delta) / dt_s;
        if (expected_hz <= 0) { score = 1.0; fired = false; return; }
        double ratio = measured_hz / expected_hz;
        if (ratio >= warn_ratio) {
            score = 1.0;
            fired = false;
        } else if (ratio <= crit_ratio) {
            score = 0.0;
            fired = true;
        } else {
            score = (ratio - crit_ratio) / (warn_ratio - crit_ratio);
            fired = true;
        }
    }
};

struct StallDetector {
    uint64_t  stall_threshold_ms{2000};
    uint64_t  disconnect_threshold_ms{10000};

    uint64_t  last_bytes_processed{0};
    int64_t   last_data_seen_ns{0};
    uint64_t  stall_duration_ms{0};
    bool      fired{false};
    double    score{1.0};
    bool      should_disconnect{false};

    void update(uint64_t bytes_processed, int64_t now_ns) {
        if (bytes_processed != last_bytes_processed) {
            last_bytes_processed = bytes_processed;
            last_data_seen_ns = now_ns;
            stall_duration_ms = 0;
            fired = false;
            should_disconnect = false;
            score = 1.0;
        } else if (last_data_seen_ns > 0) {
            stall_duration_ms = static_cast<uint64_t>(
                (now_ns - last_data_seen_ns) / 1'000'000);
            if (stall_duration_ms >= disconnect_threshold_ms) {
                should_disconnect = true;
                fired = true;
                score = 0.0;
            } else if (stall_duration_ms >= stall_threshold_ms) {
                fired = true;
                score = 1.0 - static_cast<double>(stall_duration_ms) /
                        static_cast<double>(disconnect_threshold_ms);
                if (score < 0.0) score = 0.0;
            } else {
                fired = false;
                score = 1.0;
            }
        }
    }
};

struct CrcErrorDetector {
    double   error_rate_warn{0.01};
    double   error_rate_crit{0.05};

    uint64_t prev_crc_errors{0};
    uint64_t prev_packets{0};
    double   error_rate{0};
    bool     fired{false};
    double   score{1.0};

    void update(uint64_t crc_errors, uint64_t packets_parsed) {
        uint64_t d_err = crc_errors - prev_crc_errors;
        uint64_t d_pkt = (packets_parsed - prev_packets) + d_err;
        prev_crc_errors = crc_errors;
        prev_packets = packets_parsed;

        if (d_pkt == 0) { error_rate = 0; fired = false; score = 1.0; return; }

        error_rate = static_cast<double>(d_err) / static_cast<double>(d_pkt);
        if (error_rate >= error_rate_crit) {
            fired = true;
            score = 0.0;
        } else if (error_rate >= error_rate_warn) {
            fired = true;
            score = 1.0 - (error_rate - error_rate_warn) /
                    (error_rate_crit - error_rate_warn);
        } else {
            fired = false;
            score = 1.0;
        }
    }
};

struct FlapDetector {
    uint32_t  max_reconnects_per_window{5};
    double    window_ns{60.0e9};

    static constexpr size_t kRingSize = 32;
    std::array<int64_t, kRingSize> reconnect_times{};
    size_t    ring_head{0};
    uint32_t  total_reconnects{0};
    uint32_t  reconnects_in_window{0};
    bool      fired{false};
    double    score{1.0};

    void record_reconnect(int64_t now_ns) {
        reconnect_times[ring_head % kRingSize] = now_ns;
        ++ring_head;
        ++total_reconnects;
    }

    void update(int64_t now_ns) {
        uint32_t count = 0;
        const size_t n = std::min(ring_head, kRingSize);
        for (size_t i = 0; i < n; ++i) {
            size_t idx = (ring_head - 1 - i) % kRingSize;
            if ((now_ns - reconnect_times[idx]) <= static_cast<int64_t>(window_ns)) {
                ++count;
            } else {
                break; // ring is chronological, older entries won't match
            }
        }
        reconnects_in_window = count;
        if (count >= max_reconnects_per_window) {
            fired = true;
            score = 0.0;
        } else if (max_reconnects_per_window > 0) {
            fired = false;
            score = 1.0 - static_cast<double>(count) /
                    static_cast<double>(max_reconnects_per_window);
        } else {
            fired = false;
            score = 1.0;
        }
    }
};

struct HeartbeatJitterDetector {
    double   max_jitter_ratio{2.0};
    double   expected_interval_ms{1000.0};
    double   ema_alpha{0.2};

    double   last_rtt_ms{0};
    double   ema_rtt_ms{0};
    bool     initialized{false};
    bool     fired{false};
    double   score{1.0};

    void update(double rtt_ms) {
        last_rtt_ms = rtt_ms;
        if (!initialized) {
            ema_rtt_ms = rtt_ms;
            initialized = true;
        } else {
            ema_rtt_ms = ema_alpha * rtt_ms + (1.0 - ema_alpha) * ema_rtt_ms;
        }

        double threshold = expected_interval_ms * max_jitter_ratio;
        if (ema_rtt_ms > threshold) {
            fired = true;
            score = 0.0;
        } else if (threshold > 0) {
            fired = false;
            score = 1.0 - ema_rtt_ms / threshold;
            if (score < 0) score = 0;
        }
    }

    void reset() {
        last_rtt_ms = 0;
        ema_rtt_ms = 0;
        initialized = false;
        fired = false;
        score = 1.0;
    }
};

} // namespace detail

// ─── DeviceHealthMonitor ────────────────────────────────────────────────────

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

    void start();
    void stop();
    bool is_running() const;

    // ── State queries (thread-safe) ─────────────────────────────────────

    DeviceHealthState state() const;
    DeviceHealthSnapshot snapshot() const;
    double health_score() const;
    DeviceFault active_faults() const;

    // ── Callbacks ───────────────────────────────────────────────────────

    void on_health_update(DeviceHealthUpdateCallback cb);
    void on_state_change(DeviceHealthStateCallback cb);
    void on_fault(DeviceFaultCallback cb);

    // ── Manual overrides ────────────────────────────────────────────────

    void force_state(DeviceHealthState s);
    void reset();

    // ── External event injection ────────────────────────────────────────

    /// Notify of a heartbeat RTT measurement.
    void notify_heartbeat_rtt(int64_t rtt_us);

    /// Notify of a connection event.
    void notify_connection_event(ConnectionEvent event);

    // ── Per-sensor packet counting (called from I/O thread) ─────────────

    /// Lightweight counter bumps — registered as parser callbacks.
    void count_lidar()  { lidar_count_.fetch_add(1, std::memory_order_relaxed); }
    void count_imu()    { imu_count_.fetch_add(1, std::memory_order_relaxed); }
    void count_camera() { camera_count_.fetch_add(1, std::memory_order_relaxed); }

private:
    using Clock     = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    void monitor_loop();
    void tick();
    void evaluate_state_machine(DeviceFault faults, int64_t now_ns);
    double compute_health_score() const;
    void emit_callbacks(const DeviceHealthSnapshot& snap, DeviceFault faults,
                        DeviceFault prev_faults, DeviceHealthState prev_state);

    static int64_t now_ns() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now().time_since_epoch()).count();
    }

    // --- References (non-owning) ---
    ConnectionManager&  conn_mgr_;
    PacketParser&       parser_;
    DeviceHealthConfig  cfg_;

    // --- Per-sensor atomic counters (incremented on I/O thread) ---
    std::atomic<uint64_t> lidar_count_{0};
    std::atomic<uint64_t> imu_count_{0};
    std::atomic<uint64_t> camera_count_{0};

    // --- Heartbeat RTT (set from control callback) ---
    std::atomic<int64_t> latest_heartbeat_rtt_us_{0};
    std::atomic<bool>    heartbeat_rtt_available_{false};

    // --- Detectors ---
    detail::RateDropDetector         rate_lidar_;
    detail::RateDropDetector         rate_imu_;
    detail::RateDropDetector         rate_camera_;
    detail::StallDetector            stall_;
    detail::CrcErrorDetector         crc_;
    detail::FlapDetector             flap_;
    detail::HeartbeatJitterDetector  hb_jitter_;

    // --- State machine ---
    std::atomic<DeviceHealthState> state_{DeviceHealthState::Disconnected};
    int64_t state_entered_ns_{0};
    int64_t last_fault_cleared_ns_{0};
    DeviceFault prev_faults_{DeviceFault::None};

    // --- Timing ---
    TimePoint last_tick_{Clock::now()};
    bool first_tick_{true};

    // --- Monitor thread ---
    std::atomic<bool> running_{false};
    std::thread       monitor_thread_;

    // --- Snapshot (protected by mutex) ---
    mutable std::mutex   snapshot_mu_;
    DeviceHealthSnapshot latest_snapshot_;

    // --- Callbacks (protected by mutex) ---
    std::mutex                   cb_mu_;
    DeviceHealthUpdateCallback   update_cb_;
    DeviceHealthStateCallback    state_cb_;
    DeviceFaultCallback          fault_cb_;
};

} // namespace thunderbird
