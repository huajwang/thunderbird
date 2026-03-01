// ─────────────────────────────────────────────────────────────────────────────
// Example — DeviceHealthMonitor usage
// ─────────────────────────────────────────────────────────────────────────────
//
// Demonstrates:
//   1. Creating a DeviceHealthMonitor attached to a ConnectionManager
//   2. Configuring health thresholds (DeviceHealthConfig)
//   3. Subscribing to health updates, state transitions, and fault callbacks
//   4. Querying health_score() and DeviceHealthSnapshot
//   5. Injecting simulated connection events and heartbeat RTTs
//   6. Per-sensor packet counting (count_lidar / count_imu / count_camera)
//
// This example uses a SimulatedTransport so no real hardware is needed.
//
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/thunderbird.h"
#include "thunderbird/simulated_transport.h"
#include "thunderbird/device_health_monitor.h"

#include <chrono>
#include <cstdio>
#include <thread>

using namespace thunderbird;

int main() {
    std::puts("=== DeviceHealthMonitor Demo ===\n");

    // ── 1. Set up simulated ConnectionManager ───────────────────────────────
    auto sim = std::make_unique<SimulatedTransport>();
    auto* sim_raw = sim.get();

    RetryConfig retry;
    retry.max_connect_attempts = 1;
    ConnectionConfig conn;
    conn.connect_timeout_ms   = 500;
    conn.handshake_timeout_ms = 500;

    auto conn_mgr = std::make_unique<ConnectionManager>(
        std::move(sim), conn, retry);

    // Feed handshake so connect() succeeds.
    {
        using namespace protocol;
        uint8_t pkt_buf[2048];
        HandshakeAckPayload ack{};
        ack.accepted = 1;
        ack.device_protocol_version = kProtocolVersion;
        std::strncpy(ack.serial_number,    "TB-HEALTH-001", sizeof(ack.serial_number) - 1);
        std::strncpy(ack.firmware_version, "2.0.0",         sizeof(ack.firmware_version) - 1);
        std::strncpy(ack.model_name,       "Thunderbird-X", sizeof(ack.model_name) - 1);

        size_t n = build_packet(pkt_buf, sizeof(pkt_buf),
                                PacketType::HandshakeAck, 0, 0,
                                reinterpret_cast<const uint8_t*>(&ack), sizeof(ack));
        sim_raw->inject(pkt_buf, n);
    }

    auto status = conn_mgr->connect("sim://health-demo");
    if (status != Status::OK) {
        std::fprintf(stderr, "Connection failed: %s\n", status_string(status));
        return 1;
    }

    // ── 2. Configure health monitor ─────────────────────────────────────────
    DeviceHealthConfig health_cfg;
    health_cfg.tick_hz             = 10.0;    // 10 Hz for faster demo response
    health_cfg.heal_window_ms      = 1000;    // 1 second to heal
    health_cfg.rate_warn_ratio     = 0.70;
    health_cfg.rate_crit_ratio     = 0.30;
    // Disable stall detection for this demo — we only bump counters, not
    // actual decoder bytes, so the stall detector would fire immediately.
    health_cfg.stall_threshold_ms  = 60000;
    health_cfg.stall_disconnect_threshold_ms = 120000;
    health_cfg.crc_error_rate_warn = 0.01;
    health_cfg.crc_error_rate_crit = 0.05;
    health_cfg.expected_lidar_hz   = 10.0;
    health_cfg.expected_imu_hz     = 200.0;
    health_cfg.expected_camera_fps = 30.0;

    // ── 3. Create monitor ───────────────────────────────────────────────────
    // Construct BEFORE connect so it can observe ConnectionEvent::StreamStarted.
    DeviceHealthMonitor monitor(*conn_mgr, conn_mgr->decoder(), health_cfg);
    monitor.start();

    // Now notify the monitor that streaming has begun.
    monitor.notify_connection_event(ConnectionEvent::StreamStarted);

    // ── 4. Register callbacks ───────────────────────────────────────────────

    // Health update (fires every tick)
    int update_count = 0;
    monitor.on_health_update([&update_count](const DeviceHealthSnapshot& snap) {
        ++update_count;
        if (update_count <= 3 || update_count % 10 == 0) {
            std::printf("  [health] tick=%d  state=%-12s  score=%.2f  "
                        "lidar=%.1f Hz  imu=%.1f Hz\n",
                        update_count,
                        device_health_state_name(snap.state),
                        snap.health_score,
                        snap.lidar_hz,
                        snap.imu_hz);
        }
    });

    // State transitions
    monitor.on_state_change(
        [](DeviceHealthState from, DeviceHealthState to, DeviceFault faults) {
            std::printf("  [state] %s → %s  faults=0x%04x\n",
                        device_health_state_name(from),
                        device_health_state_name(to),
                        static_cast<unsigned>(faults));
        });

    // Fault callback
    monitor.on_fault(
        [](DeviceFault fault, const DeviceHealthSnapshot& snap) {
            std::printf("  [FAULT] %s  score=%.2f\n",
                        device_fault_name(fault),
                        snap.health_score);
        });

    // ── 5. Monitor is already running ────────────────────────────────────────
    std::printf("Monitor started (tick_hz=%.0f)\n\n", health_cfg.tick_hz);

    // ── 6. Simulate healthy traffic by bumping sensor counters ──────────────
    //
    // The health monitor reads these atomics each tick to measure packet rates.
    //
    std::printf("--- Simulating 1s of healthy traffic ---\n");
    auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - t0 < std::chrono::seconds(1)) {
        // ~200 Hz IMU
        monitor.count_imu();
        // ~10 Hz LiDAR (every 20th iteration)
        static int iter = 0;
        if (++iter % 20 == 0) monitor.count_lidar();
        // ~30 Hz camera (every 7th iteration)
        if (iter % 7 == 0) monitor.count_camera();

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // ── 7. Inject heartbeat RTTs ────────────────────────────────────────────
    std::printf("\n--- Injecting heartbeat RTTs ---\n");
    for (int i = 0; i < 5; ++i) {
        int64_t rtt_us = 500 + i * 100;  // 500–900 μs
        monitor.notify_heartbeat_rtt(rtt_us);
        std::printf("  heartbeat RTT: %lld μs\n", static_cast<long long>(rtt_us));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // ── 8. Query snapshot ───────────────────────────────────────────────────
    auto snap = monitor.snapshot();
    std::printf("\n--- DeviceHealthSnapshot ---\n"
                "  state:            %s\n"
                "  health_score:     %.3f\n"
                "  lidar_hz:         %.1f\n"
                "  imu_hz:           %.1f\n"
                "  camera_fps:       %.1f\n"
                "  crc_error_rate:   %.4f\n"
                "  stall_duration:   %llu ms\n"
                "  reconnect_count:  %u\n"
                "  heartbeat_rtt:    %.1f ms\n"
                "  active_faults:    0x%04x\n",
                device_health_state_name(snap.state),
                snap.health_score,
                snap.lidar_hz,
                snap.imu_hz,
                snap.camera_fps,
                snap.crc_error_rate,
                static_cast<unsigned long long>(snap.stall_duration_ms),
                snap.reconnect_count,
                snap.heartbeat_rtt_ms,
                static_cast<unsigned>(snap.active_faults));

    // ── 9. Simulate degradation (stop sending packets) ──────────────────────
    std::printf("\n--- Simulating stall (no packets for 1s) ---\n");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    snap = monitor.snapshot();
    std::printf("After stall: state=%s  score=%.3f  stall=%llu ms  faults=0x%04x\n",
                device_health_state_name(snap.state),
                snap.health_score,
                static_cast<unsigned long long>(snap.stall_duration_ms),
                static_cast<unsigned>(snap.active_faults));

    // ── 10. Inject reconnection events ──────────────────────────────────────
    std::printf("\n--- Injecting reconnection events ---\n");
    monitor.notify_connection_event(ConnectionEvent::Disconnected);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    monitor.notify_connection_event(ConnectionEvent::Reconnecting);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    monitor.notify_connection_event(ConnectionEvent::Connected);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    snap = monitor.snapshot();
    std::printf("After reconnect: state=%s  reconnects=%u\n",
                device_health_state_name(snap.state),
                snap.reconnect_count);

    // ── 11. Fault flags inspection ──────────────────────────────────────────
    std::printf("\n--- Fault flag helpers ---\n");
    DeviceFault combined = DeviceFault::TransportStall | DeviceFault::CrcErrorSpike;
    std::printf("  combined faults: 0x%04x  count=%u\n",
                static_cast<unsigned>(combined),
                device_fault_count(combined));
    std::printf("  has TransportStall? %s\n",
                has_device_fault(combined, DeviceFault::TransportStall) ? "yes" : "no");
    std::printf("  has LidarRateDrop?  %s\n",
                has_device_fault(combined, DeviceFault::LidarRateDrop) ? "yes" : "no");

    // ── 12. Stop and cleanup ────────────────────────────────────────────────
    monitor.stop();
    conn_mgr->disconnect();

    std::printf("\nMonitor stopped. Total ticks: %d\n", update_count);
    std::puts("Done.");
    return 0;
}
