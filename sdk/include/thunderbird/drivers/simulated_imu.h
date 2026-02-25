// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Simulated IMU driver
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/sensor_driver.h"
#include <atomic>
#include <cmath>
#include <random>
#include <thread>

namespace thunderbird {

/// Generates synthetic IMU data at a configurable rate (default 200 Hz).
class SimulatedImuDriver final : public ISensorDriver {
public:
    explicit SimulatedImuDriver(ImuCallback cb, double sample_rate_hz = 200.0)
        : callback_(std::move(cb)),
          period_us_(static_cast<int>(1'000'000.0 / sample_rate_hz)) {}

    SensorType type() const override { return SensorType::IMU; }
    std::string name() const override { return "SimulatedIMU"; }

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
        std::mt19937 rng(123);
        std::normal_distribution<float> noise(0.0f, 0.02f);

        while (streaming_) {
            auto sample = std::make_shared<ImuSample>();
            sample->timestamp      = Timestamp::now();
            sample->host_timestamp = sample->timestamp;

            // Gravity + small noise on accel
            sample->accel = {noise(rng), noise(rng), 9.81f + noise(rng)};
            // Small gyro drift
            sample->gyro  = {noise(rng) * 0.01f, noise(rng) * 0.01f, noise(rng) * 0.01f};
            sample->temperature = 25.0f + noise(rng);

            if (callback_) callback_(sample);
            std::this_thread::sleep_for(std::chrono::microseconds(period_us_));
        }
    }

    ImuCallback          callback_;
    int                  period_us_;
    std::atomic<bool>    streaming_{false};
    std::thread          thread_;
};

} // namespace thunderbird
