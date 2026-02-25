// ─────────────────────────────────────────────────────────────────────────────
// Example 4 — Device communication layer usage
// ─────────────────────────────────────────────────────────────────────────────
//
// Demonstrates:
//   1. Creating a transport (Ethernet or USB)
//   2. Setting up ConnectionManager with retry policy
//   3. Registering sensor callbacks on the PacketParser
//   4. Handling connection events (retry, reconnect, heartbeat)
//   5. Streaming and receiving parsed sensor data
//
// This example uses the SimulatedTransport to work without real hardware.
// To switch to real hardware, change the transport and URI.
//
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/thunderbird.h"
#include "thunderbird/simulated_transport.h"
#include "thunderbird/ethernet_transport.h"
#include "thunderbird/usb_transport.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>

// ── Helper: feed synthetic packets into a SimulatedTransport ────────────────
// In a real scenario the device firmware sends these over Ethernet/USB.
static void feed_synthetic_packets(thunderbird::SimulatedTransport& sim,
                                   int num_packets)
{
    using namespace thunderbird::protocol;
    uint8_t pkt_buf[2048];

    // 1) Feed a HandshakeAck so the connection state machine succeeds.
    {
        HandshakeAckPayload ack{};
        ack.accepted = 1;
        ack.device_protocol_version = kProtocolVersion;
        std::strncpy(ack.serial_number,    "TB-HW-00042",   sizeof(ack.serial_number) - 1);
        std::strncpy(ack.firmware_version, "1.2.3",         sizeof(ack.firmware_version) - 1);
        std::strncpy(ack.model_name,       "Thunderbird-X", sizeof(ack.model_name) - 1);

        size_t n = build_packet(pkt_buf, sizeof(pkt_buf),
                                PacketType::HandshakeAck, 0, 0,
                                reinterpret_cast<const uint8_t*>(&ack), sizeof(ack));
        sim.inject(pkt_buf, n);
    }

    // Small delay to let the handshake be consumed
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 2) Feed IMU packets
    for (int i = 0; i < num_packets; ++i) {
        ImuWirePayload imu;
        imu.accel_x = 0.01f * i;
        imu.accel_y = 0.02f * i;
        imu.accel_z = 9.81f;
        imu.gyro_x  = 0.001f;
        imu.gyro_y  = 0.0f;
        imu.gyro_z  = 0.0f;
        imu.temperature = 25.5f;

        int64_t ts = thunderbird::Timestamp::now().nanoseconds;
        size_t n = build_packet(pkt_buf, sizeof(pkt_buf),
                                PacketType::ImuSample,
                                static_cast<uint32_t>(i), ts,
                                reinterpret_cast<const uint8_t*>(&imu), sizeof(imu));
        sim.inject(pkt_buf, n);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // 3) Feed a LiDAR packet
    {
        constexpr int kPoints = 100;
        LidarSubHeader sub;
        sub.num_points    = kPoints;
        sub.azimuth_start = 0.0f;
        sub.azimuth_end   = 360.0f;

        std::vector<LidarWirePoint> pts(kPoints);
        for (int i = 0; i < kPoints; ++i) {
            pts[i].x = static_cast<float>(i) * 0.1f;
            pts[i].y = static_cast<float>(i) * 0.2f;
            pts[i].z = 1.0f;
            pts[i].intensity = static_cast<uint8_t>(i % 256);
            pts[i].ring = static_cast<uint8_t>(i % 16);
        }

        // Build payload = sub-header + points
        std::vector<uint8_t> payload(sizeof(sub) + kPoints * sizeof(LidarWirePoint));
        std::memcpy(payload.data(), &sub, sizeof(sub));
        std::memcpy(payload.data() + sizeof(sub), pts.data(),
                    kPoints * sizeof(LidarWirePoint));

        int64_t ts = thunderbird::Timestamp::now().nanoseconds;
        size_t n = build_packet(pkt_buf, sizeof(pkt_buf),
                                PacketType::LidarScan, 0, ts,
                                payload.data(),
                                static_cast<uint32_t>(payload.size()));
        sim.inject(pkt_buf, n);
    }
}

// ─────────────────────────────────────────────────────────────────────────────

int main() {
    using namespace thunderbird;

    std::puts("=== Communication Layer Demo ===\n");

    // ── 1. Create a simulated transport (swap for EthernetTransport/UsbTransport) ──
    auto sim_transport = std::make_unique<SimulatedTransport>();
    auto* sim_raw = sim_transport.get(); // keep raw ptr for injection

    // ── 2. Configure retry policy ───────────────────────────────────────────
    RetryConfig retry;
    retry.initial_retry_delay_ms = 200;
    retry.backoff_multiplier     = 2.0;
    retry.max_retry_delay_ms     = 5000;
    retry.max_connect_attempts   = 3;
    retry.jitter_factor          = 0.1;

    ConnectionConfig conn;
    conn.connect_timeout_ms   = 2000;
    conn.handshake_timeout_ms = 1000;
    conn.heartbeat_interval_ms = 500;
    conn.heartbeat_miss_limit  = 5;

    // ── 3. Build ConnectionManager ──────────────────────────────────────────
    auto conn_mgr = std::make_unique<ConnectionManager>(
        std::move(sim_transport), conn, retry);

    // ── 4. Register event callback ──────────────────────────────────────────
    conn_mgr->on_event([](ConnectionEvent evt, const std::string& detail) {
        std::printf("[EVENT] %s: %s\n",
                    connection_event_name(evt), detail.c_str());
    });

    // ── 5. Register sensor data callbacks on the parser ─────────────────────
    std::atomic<int> imu_count{0}, lidar_count{0};

    conn_mgr->parser().on_imu([&](std::shared_ptr<const ImuSample> s) {
        int n = ++imu_count;
        if (n <= 5 || n % 10 == 0)  // print first 5 + every 10th
            std::printf("  [IMU #%d] accel=(%.3f, %.3f, %.3f)  temp=%.1f°C\n",
                        n, s->accel[0], s->accel[1], s->accel[2], s->temperature);
    });

    conn_mgr->parser().on_lidar([&](std::shared_ptr<const LidarFrame> f) {
        ++lidar_count;
        std::printf("  [LiDAR] seq=%u  points=%zu\n",
                    f->sequence_number, f->points.size());
    });

    // ── 6. Connect (with retry) ─────────────────────────────────────────────
    // Feed the handshake response before connect() blocks waiting for it.
    std::thread feeder([sim_raw] {
        feed_synthetic_packets(*sim_raw, /*num_imu=*/20);
    });

    std::printf("Connecting ...\n");
    Status s = conn_mgr->connect("sim://loopback");
    std::printf("connect() returned: %s\n", status_string(s));

    if (s == Status::OK) {
        auto info = conn_mgr->device_info();
        std::printf("Device: %s  S/N: %s  FW: %s\n",
                    info.model_name.c_str(),
                    info.serial_number.c_str(),
                    info.firmware_version.c_str());

        // ── 7. Start streaming ──────────────────────────────────────────
        conn_mgr->start_streaming();
    }

    feeder.join();

    // Let the I/O thread process remaining data.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // ── 8. Print parser stats ───────────────────────────────────────────────
    auto stats = conn_mgr->parser_stats();
    std::printf("\n── Parser stats ─────────────────────\n");
    std::printf("  packets parsed : %llu\n", (unsigned long long)stats.packets_parsed);
    std::printf("  CRC errors     : %llu\n", (unsigned long long)stats.crc_errors);
    std::printf("  resync count   : %llu\n", (unsigned long long)stats.resync_count);
    std::printf("  bytes processed: %llu\n", (unsigned long long)stats.bytes_processed);

    // ── 9. Cleanup ──────────────────────────────────────────────────────────
    conn_mgr->stop_streaming();
    conn_mgr->disconnect();

    std::printf("\nReceived: %d IMU samples, %d LiDAR frames\n",
                imu_count.load(), lidar_count.load());
    std::puts("Done.");
    return 0;
}
