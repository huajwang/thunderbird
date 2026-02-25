// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Simulated LiDAR driver
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/sensor_driver.h"
#include <atomic>
#include <cmath>
#include <random>
#include <thread>

namespace thunderbird {

/// Generates synthetic LiDAR scans at a configurable rate (default 10 Hz).
class SimulatedLidarDriver final : public ISensorDriver {
public:
    explicit SimulatedLidarDriver(LidarCallback cb, double scan_rate_hz = 10.0)
        : callback_(std::move(cb)), period_ms_(static_cast<int>(1000.0 / scan_rate_hz)) {}

    SensorType type() const override { return SensorType::LiDAR; }
    std::string name() const override { return "SimulatedLiDAR"; }

    Status initialize() override { return Status::OK; }

    Status start_streaming() override {
        if (streaming_.exchange(true)) return Status::AlreadyStreaming;
        thread_ = std::thread([this] { run(); });
        return Status::OK;
    }

    Status stop_streaming() override {
        streaming_ = false;
        if (thread_.joinable()) thread_.join();
        return Status::OK;
    }

    bool is_streaming() const override { return streaming_; }

private:
    void run() {
        std::mt19937 rng(42);
        std::uniform_real_distribution<float> dist_r(1.0f, 50.0f);
        std::uniform_real_distribution<float> dist_i(0.0f, 255.0f);
        uint32_t seq = 0;

        while (streaming_) {
            auto frame = std::make_shared<LidarFrame>();
            frame->timestamp      = Timestamp::now();
            frame->host_timestamp = frame->timestamp;
            frame->sequence_number = seq++;

            // Generate a synthetic 360° scan with 16 rings
            constexpr int kRings  = 16;
            constexpr int kCols   = 360;
            frame->points.reserve(kRings * kCols);

            for (int ring = 0; ring < kRings; ++ring) {
                float elevation = -15.0f + 2.0f * static_cast<float>(ring); // degrees
                float elev_rad  = elevation * 3.14159265f / 180.0f;
                for (int col = 0; col < kCols; ++col) {
                    float azimuth_rad = static_cast<float>(col) * 3.14159265f / 180.0f;
                    float r = dist_r(rng);

                    LidarPoint pt;
                    pt.x         = r * std::cos(elev_rad) * std::cos(azimuth_rad);
                    pt.y         = r * std::cos(elev_rad) * std::sin(azimuth_rad);
                    pt.z         = r * std::sin(elev_rad);
                    pt.intensity = dist_i(rng);
                    pt.ring      = static_cast<uint8_t>(ring);
                    frame->points.push_back(pt);
                }
            }

            if (callback_) callback_(frame);
            std::this_thread::sleep_for(std::chrono::milliseconds(period_ms_));
        }
    }

    LidarCallback       callback_;
    int                  period_ms_;
    std::atomic<bool>    streaming_{false};
    std::thread          thread_;
};

} // namespace thunderbird
