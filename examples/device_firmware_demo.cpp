// ─────────────────────────────────────────────────────────────────────────────
// Example — DeviceManager integration with real firmware
// ─────────────────────────────────────────────────────────────────────────────
//
// Uses the high-level DeviceManager API to connect to a Thunderbird device
// (or the stub firmware) over Ethernet (TCP control + UDP data).
//
// This is the recommended entry point for end-user applications.
//
// Requirements:
//   - SDK built with -DTHUNDERBIRD_USE_SIMULATED=OFF
//   - Firmware running:  ./firmware/build/thunderbird_firmware 5555
//
// Usage:
//   ./build/examples/device_firmware_demo [host] [port]
//
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/thunderbird.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

int main(int argc, const char* argv[]) {
    using namespace thunderbird;

    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";
    const char* port = (argc > 2) ? argv[2] : "5555";

    std::printf("=== DeviceManager Firmware Demo ===\n");
    std::printf("Target: eth://%s:%s\n\n", host, port);

    // ── Configure ───────────────────────────────────────────────────────
    DeviceConfig cfg;
    cfg.uri = std::string("eth://") + host + ":" + port;

    cfg.lidar_hz   = 10.0;
    cfg.imu_hz     = 200.0;
    cfg.camera_fps = 30.0;

    cfg.connection.connect_timeout_ms    = 3000;
    cfg.connection.handshake_timeout_ms  = 2000;
    cfg.connection.heartbeat_interval_ms = 1000;
    cfg.connection.heartbeat_miss_limit  = 5;

    cfg.retry.initial_retry_delay_ms = 500;
    cfg.retry.max_connect_attempts   = 3;

    // ── Create DeviceManager ────────────────────────────────────────────
    DeviceManager device(cfg);

    // ── Sensor callbacks ────────────────────────────────────────────────
    std::atomic<int> imu_n{0}, lidar_n{0}, cam_n{0};

    device.on_imu([&](std::shared_ptr<const ImuSample> s) {
        int n = ++imu_n;
        if (n <= 3 || n % 100 == 0)
            std::printf("  [IMU #%d] accel=(%.3f, %.3f, %.3f)\n",
                        n, s->accel[0], s->accel[1], s->accel[2]);
    });

    device.on_lidar([&](std::shared_ptr<const LidarFrame> f) {
        int n = ++lidar_n;
        if (n <= 3 || n % 10 == 0)
            std::printf("  [LiDAR #%d] seq=%u points=%zu\n",
                        n, f->sequence_number, f->points.size());
    });

    device.on_camera([&](std::shared_ptr<const CameraFrame> f) {
        int n = ++cam_n;
        if (n <= 3 || n % 30 == 0)
            std::printf("  [Camera #%d] %ux%u format=%u\n",
                        n, f->width, f->height,
                        static_cast<unsigned>(f->format));
    });

    // ── Connect ─────────────────────────────────────────────────────────
    std::printf("Connecting ...\n");
    Status s = device.connect();
    std::printf("connect() → %s\n", status_string(s));
    if (s != Status::OK) {
        std::fprintf(stderr, "Failed to connect. Is the firmware running?\n");
        return 1;
    }

    auto info = device.device_info();
    std::printf("\nDevice: %s  S/N: %s  FW: %s\n\n",
                info.model_name.c_str(),
                info.serial_number.c_str(),
                info.firmware_version.c_str());

    // ── Stream ──────────────────────────────────────────────────────────
    s = device.start();
    std::printf("start() → %s\n", status_string(s));
    if (s != Status::OK && s != Status::AlreadyStreaming) {
        std::fprintf(stderr, "Failed to start streaming.\n");
        device.disconnect();
        return 1;
    }

    std::printf("Streaming for 5 seconds ...\n\n");
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // ── Stop & report ───────────────────────────────────────────────────
    device.stop();

    std::printf("\n── Results ─────────────────────────────\n");
    std::printf("  IMU samples  : %d\n", imu_n.load());
    std::printf("  LiDAR scans  : %d\n", lidar_n.load());
    std::printf("  Camera frames: %d\n", cam_n.load());

    // Device health
    auto health = device.device_health();
    std::printf("  Health state : %s\n",
                device_health_state_name(health));

    // Diagnostics snapshot
    auto diag = device.diagnostics_snapshot();
    std::printf("  Diagnostics  : %zu groups collected\n", diag.groups.size());
    std::printf("────────────────────────────────────────\n");

    device.disconnect();
    std::printf("Done.\n");
    return 0;
}
