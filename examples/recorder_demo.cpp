// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Recorder / Player demo
// ─────────────────────────────────────────────────────────────────────────────
//
// Demonstrates the complete record-then-playback workflow:
//
//   1.  Generate synthetic LiDAR, IMU, and Camera frames.
//   2.  Record them to "demo_session.tbrec" using Recorder.
//   3.  Play the file back with Player (Pull API) and print each frame.
//   4.  Play the file back with Player (Callback API) at 2× speed.
//
// Build:
//   cmake -S . -B build -DTHUNDERBIRD_BUILD_EXAMPLES=ON
//         -DTHUNDERBIRD_USE_SIMULATED=ON
//   cmake --build build --target recorder_demo
//   ./build/examples/recorder_demo
//
// ─────────────────────────────────────────────────────────────────────────────

#include "thunderbird/recorder.h"
#include "thunderbird/player.h"
#include "thunderbird/sensor_data.h"

#include <chrono>
#include <cstdio>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

namespace tbd = thunderbird::data;

// ─── Synthetic data generators ──────────────────────────────────────────────

static tbd::LidarFrame make_lidar(int64_t ts, uint32_t seq, int npts) {
    tbd::LidarFrame f;
    f.timestamp_ns = ts;
    f.sequence     = seq;
    f.points.resize(static_cast<std::size_t>(npts));
    for (int i = 0; i < npts; ++i) {
        auto& p     = f.points[static_cast<std::size_t>(i)];
        p.x         = static_cast<float>(i) * 0.1f;
        p.y         = static_cast<float>(i) * 0.2f;
        p.z         = static_cast<float>(i) * 0.05f;
        p.intensity = static_cast<float>(i % 256);
        p.timestamp_offset_ns = i * 100;
    }
    return f;
}

static tbd::ImuFrame make_imu(int64_t ts) {
    tbd::ImuFrame f;
    f.timestamp_ns = ts;
    f.accel = {0.01f, 0.02f, 9.81f};
    f.gyro  = {0.001f, -0.002f, 0.003f};
    return f;
}

static tbd::ImageFrame make_camera(int64_t ts, uint32_t seq) {
    tbd::ImageFrame f;
    f.timestamp_ns = ts;
    f.width    = 320;
    f.height   = 240;
    f.stride   = 320 * 3;
    f.format   = tbd::PixelFormat::RGB8;
    f.sequence = seq;
    auto buf   = std::make_shared<std::vector<uint8_t>>(f.stride * f.height);
    for (auto& b : *buf) b = static_cast<uint8_t>(seq & 0xFF);
    f.data = std::move(buf);
    return f;
}

// ─── Main ───────────────────────────────────────────────────────────────────

int main() {
    const char* filename = "demo_session.tbrec";

    // ── Phase 1: Record ─────────────────────────────────────────────────
    std::printf("=== Phase 1: Recording to %s ===\n", filename);
    {
        tbd::Recorder rec(filename,
                          {.serial_number   = "TB-DEMO-001",
                           .firmware_version = "1.5.0",
                           .model_name       = "Thunderbird-X1"});
        rec.start();

        constexpr int FRAMES = 20;
        int64_t ts = 1'000'000'000;  // start at 1 second in ns

        for (int i = 0; i < FRAMES; ++i) {
            // 10 Hz LiDAR
            rec.recordLidarFrame(make_lidar(ts, static_cast<uint32_t>(i), 100));

            // 200 Hz IMU — 20 samples per LiDAR frame
            for (int j = 0; j < 20; ++j) {
                rec.recordImuFrame(make_imu(ts + j * 500'000));
            }

            // 30 Hz Camera — 3 per LiDAR frame
            for (int j = 0; j < 3; ++j) {
                rec.recordImageFrame(
                    make_camera(ts + j * 3'333'333,
                                static_cast<uint32_t>(i * 3 + j)));
            }

            ts += 100'000'000;  // 100 ms per LiDAR frame
        }

        rec.stop();
        auto s = rec.stats();
        std::printf("  Recorded %llu LiDAR, %llu IMU, %llu Camera frames "
                    "(%llu bytes)\n",
                    static_cast<unsigned long long>(s.lidar_frames),
                    static_cast<unsigned long long>(s.imu_frames),
                    static_cast<unsigned long long>(s.camera_frames),
                    static_cast<unsigned long long>(s.total_bytes));
    }

    // ── Phase 2: Pull-API playback (as-fast-as-possible) ────────────────
    std::printf("\n=== Phase 2: Pull-API Playback (max speed) ===\n");
    {
        tbd::Player player(filename,
                           {.playback_speed = 0.0,   // max speed
                            .queue_depth    = 128});
        player.start();

        auto info = player.deviceInfo();
        std::printf("  Device: serial=%s  fw=%s  model=%s\n",
                    info.serial_number.c_str(),
                    info.firmware_version.c_str(),
                    info.model_name.c_str());

        uint64_t lidar_n = 0, imu_n = 0, cam_n = 0;

        while (!player.finished() || true) {
            bool got_any = false;
            while (auto f = player.getNextLidarFrame()) {
                ++lidar_n;
                got_any = true;
            }
            while (auto f = player.getNextImuFrame()) {
                ++imu_n;
                got_any = true;
            }
            while (auto f = player.getNextImageFrame()) {
                ++cam_n;
                got_any = true;
            }
            if (player.finished() && !got_any) break;
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }

        player.stop();
        auto s = player.stats();
        std::printf("  Pulled: %llu LiDAR, %llu IMU, %llu Camera  "
                    "(player stats: %llu total)\n",
                    static_cast<unsigned long long>(lidar_n),
                    static_cast<unsigned long long>(imu_n),
                    static_cast<unsigned long long>(cam_n),
                    static_cast<unsigned long long>(s.total_records));
    }

    // ── Phase 3: Callback-API playback at 2× speed ──────────────────────
    std::printf("\n=== Phase 3: Callback-API Playback (2x speed) ===\n");
    {
        tbd::Player player(filename,
                           {.playback_speed = 2.0});

        uint64_t lidar_n = 0, imu_n = 0, cam_n = 0;

        player.onLidarFrame([&](const tbd::LidarFrame& f) {
            ++lidar_n;
            if (lidar_n <= 3)
                std::printf("    LiDAR seq=%u  pts=%zu  ts=%lld\n",
                            f.sequence, f.points.size(),
                            static_cast<long long>(f.timestamp_ns));
        });
        player.onImuFrame([&](const tbd::ImuFrame& f) {
            ++imu_n;
            (void)f;
        });
        player.onImageFrame([&](const tbd::ImageFrame& f) {
            ++cam_n;
            (void)f;
        });

        player.start();

        // Wait for playback to finish.
        while (!player.finished()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        player.stop();
        std::printf("  Callbacks received: %llu LiDAR, %llu IMU, %llu Camera\n",
                    static_cast<unsigned long long>(lidar_n),
                    static_cast<unsigned long long>(imu_n),
                    static_cast<unsigned long long>(cam_n));
    }

    // Clean up the temporary file.
    std::remove(filename);
    std::printf("\nDone.\n");
    return 0;
}
