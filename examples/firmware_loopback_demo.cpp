// ─────────────────────────────────────────────────────────────────────────────
// Example — Firmware loopback integration test
// ─────────────────────────────────────────────────────────────────────────────
//
// Connects the SDK to the Thunderbird stub firmware running on localhost.
//
// Usage:
//   1. Start firmware:   ./firmware/build/thunderbird_firmware 5555
//   2. Run this demo:    ./build/examples/firmware_loopback_demo [host] [port]
//
// The firmware uses TCP for control and UDP (port+1) for sensor data,
// matching the SDK's EthernetTransport dual-channel architecture.
//
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/thunderbird.h"
#include "thunderbird/transport.h"
#include "thunderbird/ethernet_transport.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

// ─── Main ───────────────────────────────────────────────────────────────────

int main(int argc, const char* argv[]) {
    using namespace thunderbird;

    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";
    const char* port = (argc > 2) ? argv[2] : "5555";

    std::printf("=== Firmware Loopback Demo ===\n");
    std::printf("Target: eth://%s:%s (TCP control + UDP data on port %d)\n\n",
                host, port, std::atoi(port) + 1);

    // ── 1. Create dual-channel transport (TCP + UDP) ────────────────────
    auto transport = std::make_unique<EthernetTransport>();

    // ── 2. Configure connection ─────────────────────────────────────────
    ConnectionConfig conn;
    conn.connect_timeout_ms    = 3000;
    conn.handshake_timeout_ms  = 2000;
    conn.heartbeat_interval_ms = 1000;
    conn.heartbeat_miss_limit  = 5;

    RetryConfig retry;
    retry.initial_retry_delay_ms = 500;
    retry.max_connect_attempts   = 3;

    // ── 3. Build ConnectionManager ──────────────────────────────────────
    auto mgr = std::make_unique<ConnectionManager>(
        std::move(transport), conn, retry);

    // ── 4. Event callback ───────────────────────────────────────────────
    mgr->on_event([](ConnectionEvent evt, const std::string& detail) {
        std::printf("[EVENT] %s: %s\n",
                    connection_event_name(evt), detail.c_str());
    });

    // ── 5. Sensor data callbacks ────────────────────────────────────────
    std::atomic<int> imu_n{0}, lidar_n{0}, cam_n{0};

    mgr->decoder().on_imu([&](std::shared_ptr<const ImuSample> s) {
        int n = ++imu_n;
        if (n <= 5 || n % 50 == 0)
            std::printf("  [IMU #%d] accel=(%.3f, %.3f, %.3f) gyro=(%.4f, %.4f, %.4f) temp=%.1f°C\n",
                        n, s->accel[0], s->accel[1], s->accel[2],
                        s->gyro[0], s->gyro[1], s->gyro[2], s->temperature);
    });

    mgr->decoder().on_lidar([&](std::shared_ptr<const LidarFrame> f) {
        int n = ++lidar_n;
        if (n <= 3 || n % 10 == 0)
            std::printf("  [LiDAR #%d] seq=%u points=%zu\n",
                        n, f->sequence_number, f->points.size());
    });

    mgr->decoder().on_camera([&](std::shared_ptr<const CameraFrame> f) {
        int n = ++cam_n;
        if (n <= 3 || n % 10 == 0)
            std::printf("  [Camera #%d] seq=%u %ux%u format=%u\n",
                        n, f->sequence_number, f->width, f->height,
                        static_cast<unsigned>(f->format));
    });

    // ── 6. Connect to firmware ──────────────────────────────────────────
    std::string uri = std::string("eth://") + host + ":" + port;
    std::printf("Connecting to %s ...\n", uri.c_str());
    Status s = mgr->connect(uri);
    std::printf("connect() → %s\n", status_string(s));

    if (s != Status::OK) {
        std::fprintf(stderr, "Failed to connect. Is the firmware running?\n");
        return 1;
    }

    auto info = mgr->device_info();
    std::printf("\nDevice: %s  S/N: %s  FW: %s\n\n",
                info.model_name.c_str(),
                info.serial_number.c_str(),
                info.firmware_version.c_str());

    // ── 7. Stream for a few seconds ─────────────────────────────────────
    s = mgr->start_streaming();
    std::printf("start_streaming() → %s\n", status_string(s));
    if (s != Status::OK) {
        std::fprintf(stderr, "Failed to start streaming.\n");
        mgr->disconnect();
        return 1;
    }
    std::printf("Streaming for 5 seconds ...\n\n");
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // ── 8. Stop and print stats ─────────────────────────────────────────
    s = mgr->stop_streaming();
    std::printf("stop_streaming() → %s\n", status_string(s));
    if (s != Status::OK) {
        std::fprintf(stderr, "Failed to stop streaming.\n");
        mgr->disconnect();
        return 1;
    }

    auto stats = mgr->decoder_stats();
    std::printf("\n── Results ─────────────────────────────\n");
    std::printf("  IMU samples  : %d\n", imu_n.load());
    std::printf("  LiDAR scans  : %d\n", lidar_n.load());
    std::printf("  Camera frames: %d\n", cam_n.load());
    std::printf("  Packets parsed  : %llu\n", (unsigned long long)stats.packets_parsed);
    std::printf("  Checksum errors : %llu\n", (unsigned long long)stats.checksum_errors);
    std::printf("  Malformed       : %llu\n", (unsigned long long)stats.malformed_count);
    std::printf("  Bytes processed : %llu\n", (unsigned long long)stats.bytes_processed);
    std::printf("────────────────────────────────────────\n");

    mgr->disconnect();
    std::printf("Done.\n");
    return 0;
}
