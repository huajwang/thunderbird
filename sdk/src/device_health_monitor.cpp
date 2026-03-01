// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Device health monitor implementation
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/device_health_monitor.h"

#include <algorithm>
#include <cmath>

namespace thunderbird {

// ─── Construction / destruction ─────────────────────────────────────────────

DeviceHealthMonitor::DeviceHealthMonitor(ConnectionManager& conn_mgr,
                                         IPacketDecoder& decoder,
                                         DeviceHealthConfig config)
    : conn_mgr_(conn_mgr)
    , decoder_(decoder)
    , cfg_(std::move(config))
{
    // Initialise rate detectors.
    rate_lidar_.expected_hz = cfg_.expected_lidar_hz;
    rate_lidar_.warn_ratio  = cfg_.rate_warn_ratio;
    rate_lidar_.crit_ratio  = cfg_.rate_crit_ratio;

    rate_imu_.expected_hz = cfg_.expected_imu_hz;
    rate_imu_.warn_ratio  = cfg_.rate_warn_ratio;
    rate_imu_.crit_ratio  = cfg_.rate_crit_ratio;

    rate_camera_.expected_hz = cfg_.expected_camera_fps;
    rate_camera_.warn_ratio  = cfg_.rate_warn_ratio;
    rate_camera_.crit_ratio  = cfg_.rate_crit_ratio;

    // Initialise stall detector.
    stall_.stall_threshold_ms      = cfg_.stall_threshold_ms;
    stall_.disconnect_threshold_ms = cfg_.stall_disconnect_threshold_ms;

    // Initialise CRC error detector.
    crc_.error_rate_warn = cfg_.crc_error_rate_warn;
    crc_.error_rate_crit = cfg_.crc_error_rate_crit;

    // Initialise flap detector.
    flap_.max_reconnects_per_window = cfg_.max_reconnects_per_window;
    flap_.window_ns = cfg_.flap_window_s * 1.0e9;

    // Initialise heartbeat jitter detector.
    hb_jitter_.max_jitter_ratio     = cfg_.heartbeat_jitter_ratio;
    hb_jitter_.expected_interval_ms = 1000.0; // default heartbeat interval

    // Wire connection events from the ConnectionManager.
    conn_mgr_.on_event([this](ConnectionEvent e, const std::string& /*detail*/) {
        enqueue_connection_event(e);
    });
}

DeviceHealthMonitor::~DeviceHealthMonitor() {
    // Unregister our callback before destroying `this` to prevent
    // use-after-free if ConnectionManager outlives us.
    conn_mgr_.on_event(nullptr);
    stop();
}

// ─── Lifecycle ──────────────────────────────────────────────────────────────

void DeviceHealthMonitor::start() {
    if (running_.exchange(true)) return;  // already running
    first_tick_ = true;
    last_tick_ = Clock::now();
    monitor_thread_ = std::thread([this] { monitor_loop(); });
}

void DeviceHealthMonitor::stop() {
    running_ = false;
    if (monitor_thread_.joinable()) monitor_thread_.join();
}

bool DeviceHealthMonitor::is_running() const {
    return running_.load(std::memory_order_acquire);
}

// ─── State queries ──────────────────────────────────────────────────────────

DeviceHealthState DeviceHealthMonitor::state() const {
    return state_.load(std::memory_order_acquire);
}

DeviceHealthSnapshot DeviceHealthMonitor::snapshot() const {
    std::lock_guard<std::mutex> lk(snapshot_mu_);
    return latest_snapshot_;
}

double DeviceHealthMonitor::health_score() const {
    std::lock_guard<std::mutex> lk(snapshot_mu_);
    return latest_snapshot_.health_score;
}

DeviceFault DeviceHealthMonitor::active_faults() const {
    std::lock_guard<std::mutex> lk(snapshot_mu_);
    return latest_snapshot_.active_faults;
}

// ─── Callbacks ──────────────────────────────────────────────────────────────

void DeviceHealthMonitor::on_health_update(DeviceHealthUpdateCallback cb) {
    std::lock_guard<std::mutex> lk(cb_mu_);
    update_cb_ = std::move(cb);
}

void DeviceHealthMonitor::on_state_change(DeviceHealthStateCallback cb) {
    std::lock_guard<std::mutex> lk(cb_mu_);
    state_cb_ = std::move(cb);
}

void DeviceHealthMonitor::on_fault(DeviceFaultCallback cb) {
    std::lock_guard<std::mutex> lk(cb_mu_);
    fault_cb_ = std::move(cb);
}

// ─── Manual overrides ───────────────────────────────────────────────────────

void DeviceHealthMonitor::force_state(DeviceHealthState s) {
    state_.store(s, std::memory_order_release);
    state_entered_ns_ = now_ns();
}

void DeviceHealthMonitor::reset() {
    // Reset all detectors.
    rate_lidar_  = detail::RateDropDetector{cfg_.expected_lidar_hz,
                                            cfg_.rate_warn_ratio,
                                            cfg_.rate_crit_ratio};
    rate_imu_    = detail::RateDropDetector{cfg_.expected_imu_hz,
                                            cfg_.rate_warn_ratio,
                                            cfg_.rate_crit_ratio};
    rate_camera_ = detail::RateDropDetector{cfg_.expected_camera_fps,
                                            cfg_.rate_warn_ratio,
                                            cfg_.rate_crit_ratio};

    stall_ = detail::StallDetector{cfg_.stall_threshold_ms,
                                    cfg_.stall_disconnect_threshold_ms};
    crc_   = detail::CrcErrorDetector{cfg_.crc_error_rate_warn,
                                       cfg_.crc_error_rate_crit};
    flap_  = detail::FlapDetector{};
    flap_.max_reconnects_per_window = cfg_.max_reconnects_per_window;
    flap_.window_ns = cfg_.flap_window_s * 1.0e9;

    hb_jitter_.reset();

    // Reset counters.
    lidar_count_.store(0, std::memory_order_relaxed);
    imu_count_.store(0, std::memory_order_relaxed);
    camera_count_.store(0, std::memory_order_relaxed);
    heartbeat_rtt_available_.store(false, std::memory_order_relaxed);
    latest_heartbeat_rtt_us_.store(0, std::memory_order_relaxed);

    // Reset state.
    prev_faults_ = DeviceFault::None;
    state_.store(DeviceHealthState::Disconnected, std::memory_order_release);
    state_entered_ns_ = now_ns();
    last_fault_cleared_ns_ = 0;
    first_tick_ = true;

    {
        std::lock_guard<std::mutex> lk(snapshot_mu_);
        latest_snapshot_ = DeviceHealthSnapshot{};
    }
}

// ─── External event injection ───────────────────────────────────────────────

void DeviceHealthMonitor::notify_heartbeat_rtt(int64_t rtt_us) {
    latest_heartbeat_rtt_us_.store(rtt_us, std::memory_order_release);
    heartbeat_rtt_available_.store(true, std::memory_order_release);
}

void DeviceHealthMonitor::notify_connection_event(ConnectionEvent event) {
    // Deliberately a no-op in the new design — events flow through the
    // lock-free queue via enqueue_connection_event() instead.
    // Kept for ABI compatibility; direct callers are safe because the
    // method body below is identical to draining one event.
    process_connection_event(event);
}

void DeviceHealthMonitor::enqueue_connection_event(ConnectionEvent event) {
    std::lock_guard<std::mutex> lk(event_queue_mu_);
    pending_conn_events_.push_back(event);
}

void DeviceHealthMonitor::process_connection_event(ConnectionEvent event) {
    switch (event) {
        case ConnectionEvent::Connected:
            // Don't transition to Connected yet — wait for first data packets.
            break;

        case ConnectionEvent::StreamStarted:
            // Transition from Disconnected → Connected on first streaming.
            if (state() == DeviceHealthState::Disconnected) {
                state_.store(DeviceHealthState::Connected, std::memory_order_release);
                state_entered_ns_ = now_ns();
            }
            break;

        case ConnectionEvent::Disconnected:
        case ConnectionEvent::ReconnectFailed:
            state_.store(DeviceHealthState::Disconnected, std::memory_order_release);
            state_entered_ns_ = now_ns();
            break;

        case ConnectionEvent::Reconnecting:
            flap_.record_reconnect(now_ns());
            break;

        case ConnectionEvent::HeartbeatTimeout:
            // ConnectionManager will handle the reconnect; we note the event.
            break;

        case ConnectionEvent::StreamStopped:
        case ConnectionEvent::ProtocolError:
            break;
    }
}

// ─── Monitor loop ───────────────────────────────────────────────────────────

void DeviceHealthMonitor::monitor_loop() {
    const double tick_interval_s = 1.0 / std::max(cfg_.tick_hz, 0.1);
    const auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(tick_interval_s));

    while (running_.load(std::memory_order_acquire)) {
        tick();
        std::this_thread::sleep_for(interval);
    }
}

void DeviceHealthMonitor::tick() {
    // ── 0. Drain queued connection events (serialises with detectors) ────
    {
        std::vector<ConnectionEvent> events;
        {
            std::lock_guard<std::mutex> lk(event_queue_mu_);
            events.swap(pending_conn_events_);
        }
        for (auto e : events) {
            process_connection_event(e);
        }
    }

    const auto now_tp = Clock::now();
    const int64_t ts_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now_tp.time_since_epoch()).count();

    // ── 1. Compute elapsed time since last tick ─────────────────────────
    const double dt_s = std::chrono::duration<double>(now_tp - last_tick_).count();
    last_tick_ = now_tp;

    // Skip the first tick (no deltas available).
    if (first_tick_) {
        first_tick_ = false;
        // Seed stall detector with current byte count.
        auto dstats = decoder_.stats();
        stall_.last_bytes_processed = dstats.bytes_processed;
        stall_.last_data_seen_ns = ts_ns;
        crc_.prev_crc_errors = dstats.checksum_errors;
        crc_.prev_packets = dstats.packets_parsed;
        return;
    }

    if (dt_s <= 0) return;

    // ── 2. Sample atomic counters (exchange-to-zero) ────────────────────
    const uint64_t lidar_delta  = lidar_count_.exchange(0, std::memory_order_relaxed);
    const uint64_t imu_delta    = imu_count_.exchange(0, std::memory_order_relaxed);
    const uint64_t camera_delta = camera_count_.exchange(0, std::memory_order_relaxed);

    // ── 3. Sample DecoderStats ───────────────────────────────────────────
    const DecoderStats dstats = decoder_.stats();

    // ── 4. Run detectors ────────────────────────────────────────────────
    if (cfg_.lidar_enabled)  rate_lidar_.update(lidar_delta, dt_s);
    if (cfg_.imu_enabled)    rate_imu_.update(imu_delta, dt_s);
    if (cfg_.camera_enabled) rate_camera_.update(camera_delta, dt_s);

    stall_.update(dstats.bytes_processed, ts_ns);
    crc_.update(dstats.checksum_errors, dstats.packets_parsed);
    flap_.update(ts_ns);

    // Heartbeat RTT detector.
    if (heartbeat_rtt_available_.exchange(false, std::memory_order_acquire)) {
        double rtt_ms = static_cast<double>(
            latest_heartbeat_rtt_us_.load(std::memory_order_relaxed)) / 1000.0;
        hb_jitter_.update(rtt_ms);
    }

    // ── 5. Aggregate faults ─────────────────────────────────────────────
    DeviceFault faults = DeviceFault::None;
    if (cfg_.lidar_enabled  && rate_lidar_.fired)  faults |= DeviceFault::LidarRateDrop;
    if (cfg_.imu_enabled    && rate_imu_.fired)    faults |= DeviceFault::ImuRateDrop;
    if (cfg_.camera_enabled && rate_camera_.fired) faults |= DeviceFault::CameraRateDrop;
    if (stall_.fired)     faults |= DeviceFault::TransportStall;
    if (crc_.fired)       faults |= DeviceFault::CrcErrorSpike;
    if (flap_.fired)      faults |= DeviceFault::ReconnectFlap;
    if (hb_jitter_.fired) faults |= DeviceFault::HeartbeatJitter;

    // ── 6. State machine ────────────────────────────────────────────────
    const DeviceHealthState prev_state = state();
    evaluate_state_machine(faults, ts_ns);

    // ── 7. Build snapshot ───────────────────────────────────────────────
    DeviceHealthSnapshot snap;
    snap.state            = state();
    snap.timestamp_ns     = ts_ns;
    snap.state_entered_ns = state_entered_ns_;
    snap.lidar_hz         = rate_lidar_.measured_hz;
    snap.imu_hz           = rate_imu_.measured_hz;
    snap.camera_fps       = rate_camera_.measured_hz;
    snap.lidar_rate_score = rate_lidar_.score;
    snap.imu_rate_score   = rate_imu_.score;
    snap.camera_rate_score = rate_camera_.score;
    snap.bytes_total      = dstats.bytes_processed;
    snap.packets_total    = dstats.packets_parsed;
    snap.crc_errors_total = dstats.checksum_errors;
    snap.crc_error_rate   = crc_.error_rate;
    snap.stall_duration_ms = stall_.stall_duration_ms;
    snap.stall_score      = stall_.score;
    snap.reconnect_count  = flap_.total_reconnects;
    snap.reconnects_in_window = flap_.reconnects_in_window;
    snap.flap_detected    = flap_.fired;
    snap.heartbeat_rtt_ms     = hb_jitter_.last_rtt_ms;
    snap.heartbeat_rtt_ema_ms = hb_jitter_.ema_rtt_ms;
    snap.heartbeat_jitter     = hb_jitter_.fired;
    snap.connection_state = conn_mgr_.state();
    snap.health_score     = compute_health_score();
    snap.active_faults    = faults;
    snap.device_info      = conn_mgr_.device_info();

    {
        std::lock_guard<std::mutex> lk(snapshot_mu_);
        latest_snapshot_ = snap;
    }

    // ── 8. Callbacks ────────────────────────────────────────────────────
    emit_callbacks(snap, faults, prev_faults_, prev_state);
    prev_faults_ = faults;
}

// ─── State machine evaluation ───────────────────────────────────────────────

void DeviceHealthMonitor::evaluate_state_machine(DeviceFault faults,
                                                  int64_t t_ns) {
    const auto current = state();
    const bool any_fault = (faults != DeviceFault::None);

    switch (current) {

    case DeviceHealthState::Disconnected:
        // Transition to Connected is handled by notify_connection_event().
        break;

    case DeviceHealthState::Connected:
        if (stall_.should_disconnect) {
            state_.store(DeviceHealthState::Disconnected, std::memory_order_release);
            state_entered_ns_ = t_ns;
        } else if (any_fault) {
            state_.store(DeviceHealthState::Degraded, std::memory_order_release);
            state_entered_ns_ = t_ns;
            last_fault_cleared_ns_ = 0;
        }
        break;

    case DeviceHealthState::Degraded:
        if (stall_.should_disconnect) {
            state_.store(DeviceHealthState::Disconnected, std::memory_order_release);
            state_entered_ns_ = t_ns;
        } else if (!any_fault) {
            // Start heal timer if not already started.
            if (last_fault_cleared_ns_ == 0) {
                last_fault_cleared_ns_ = t_ns;
            }
            int64_t clear_duration_ms = (t_ns - last_fault_cleared_ns_) / 1'000'000;
            if (clear_duration_ms >= static_cast<int64_t>(cfg_.heal_window_ms)) {
                state_.store(DeviceHealthState::Connected, std::memory_order_release);
                state_entered_ns_ = t_ns;
                last_fault_cleared_ns_ = 0;
            }
        } else {
            // Faults still active — reset heal timer.
            last_fault_cleared_ns_ = 0;
        }
        break;
    }
}

// ─── Health score computation ───────────────────────────────────────────────

double DeviceHealthMonitor::compute_health_score() const {
    // Weights for each detector.
    double w_lidar  = cfg_.lidar_enabled  ? 0.25 : 0.0;
    double w_imu    = cfg_.imu_enabled    ? 0.25 : 0.0;
    double w_camera = cfg_.camera_enabled ? 0.10 : 0.0;
    double w_stall  = 0.15;
    double w_crc    = 0.10;
    double w_flap   = 0.10;
    double w_hb     = 0.05;

    double total_weight = w_lidar + w_imu + w_camera + w_stall +
                          w_crc + w_flap + w_hb;
    if (total_weight <= 0) return 1.0;

    double score = (w_lidar  * rate_lidar_.score  +
                    w_imu    * rate_imu_.score    +
                    w_camera * rate_camera_.score +
                    w_stall  * stall_.score       +
                    w_crc    * crc_.score         +
                    w_flap   * flap_.score        +
                    w_hb     * hb_jitter_.score) / total_weight;

    return std::max(0.0, std::min(1.0, score));
}

// ─── Callback emission ─────────────────────────────────────────────────────

void DeviceHealthMonitor::emit_callbacks(const DeviceHealthSnapshot& snap,
                                          DeviceFault faults,
                                          DeviceFault prev_faults,
                                          DeviceHealthState prev_state) {
    DeviceHealthUpdateCallback update_cb;
    DeviceHealthStateCallback  state_cb;
    DeviceFaultCallback        fault_cb;

    {
        std::lock_guard<std::mutex> lk(cb_mu_);
        update_cb = update_cb_;
        state_cb  = state_cb_;
        fault_cb  = fault_cb_;
    }

    // Fire update callback every tick.
    if (update_cb) {
        update_cb(snap);
    }

    // Fire state change callback on transitions.
    if (state_cb && snap.state != prev_state) {
        state_cb(prev_state, snap.state, faults);
    }

    // Fire fault callback on rising edges.
    if (fault_cb) {
        // Find newly-fired faults (present now, absent before).
        uint32_t new_faults = static_cast<uint32_t>(faults) &
                              ~static_cast<uint32_t>(prev_faults);
        if (new_faults != 0) {
            // Iterate individual fault bits.
            for (uint32_t bit = 1; bit != 0; bit <<= 1) {
                if (new_faults & bit) {
                    fault_cb(static_cast<DeviceFault>(bit), snap);
                }
            }
        }
    }
}

} // namespace thunderbird
