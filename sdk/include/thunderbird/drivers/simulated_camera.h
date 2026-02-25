// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Simulated Camera driver
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/sensor_driver.h"
#include <atomic>
#include <cmath>
#include <random>
#include <thread>

namespace thunderbird {

/// Generates synthetic camera frames at a configurable rate (default 30 Hz).
/// Produces a moving gradient pattern for visual verification.
class SimulatedCameraDriver final : public ISensorDriver {
public:
    explicit SimulatedCameraDriver(CameraCallback cb,
                                    uint32_t width = 640,
                                    uint32_t height = 480,
                                    double fps = 30.0)
        : callback_(std::move(cb)), width_(width), height_(height),
          period_ms_(static_cast<int>(1000.0 / fps)) {}

    SensorType type() const override { return SensorType::Camera; }
    std::string name() const override { return "SimulatedCamera"; }

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
        uint32_t seq = 0;
        const uint32_t stride = width_ * 3; // RGB8

        while (streaming_) {
            auto frame = std::make_shared<CameraFrame>();
            frame->timestamp      = Timestamp::now();
            frame->host_timestamp = frame->timestamp;
            frame->sequence_number = seq;
            frame->width  = width_;
            frame->height = height_;
            frame->format = PixelFormat::RGB8;
            frame->stride = stride;

            frame->data.resize(stride * height_);

            // Moving gradient pattern
            for (uint32_t y = 0; y < height_; ++y) {
                for (uint32_t x = 0; x < width_; ++x) {
                    size_t idx = (y * stride) + x * 3;
                    frame->data[idx + 0] = static_cast<uint8_t>((x + seq * 2) % 256);    // R
                    frame->data[idx + 1] = static_cast<uint8_t>((y + seq * 3) % 256);    // G
                    frame->data[idx + 2] = static_cast<uint8_t>((x + y + seq) % 256);    // B
                }
            }

            ++seq;
            if (callback_) callback_(frame);
            std::this_thread::sleep_for(std::chrono::milliseconds(period_ms_));
        }
    }

    CameraCallback       callback_;
    uint32_t             width_;
    uint32_t             height_;
    int                  period_ms_;
    std::atomic<bool>    streaming_{false};
    std::thread          thread_;
};

} // namespace thunderbird
