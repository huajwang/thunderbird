// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Simulated loopback transport
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/transport.h"
#include <mutex>
#include <queue>
#include <condition_variable>

namespace thunderbird {

/// In-process loopback transport used for PoC testing without real hardware.
/// Any data pushed via `inject()` becomes available on the read side.
class SimulatedTransport final : public ITransport {
public:
    Status open(const std::string& /*uri*/) override {
        std::lock_guard<std::mutex> lk(mu_);
        open_ = true;
        return Status::OK;
    }

    void close() override {
        std::lock_guard<std::mutex> lk(mu_);
        open_ = false;
        cv_.notify_all();
    }

    bool is_open() const override {
        std::lock_guard<std::mutex> lk(mu_);
        return open_;
    }

    size_t read(uint8_t* buf, size_t max_bytes, uint32_t timeout_ms) override {
        std::unique_lock<std::mutex> lk(mu_);
        if (!cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                          [&] { return !fifo_.empty() || !open_; })) {
            return 0; // timeout
        }
        if (!open_) return 0;

        size_t copied = 0;
        while (copied < max_bytes && !fifo_.empty()) {
            buf[copied++] = fifo_.front();
            fifo_.pop();
        }
        return copied;
    }

    size_t write(const uint8_t* buf, size_t len) override {
        // In simulation we just echo back (or discard).
        (void)buf; (void)len;
        return len;
    }

    /// Inject raw bytes that will be returned by the next `read()`.
    void inject(const uint8_t* data, size_t len) {
        std::lock_guard<std::mutex> lk(mu_);
        for (size_t i = 0; i < len; ++i) fifo_.push(data[i]);
        cv_.notify_one();
    }

private:
    mutable std::mutex      mu_;
    std::condition_variable cv_;
    std::queue<uint8_t>     fifo_;
    bool                    open_{false};
};

} // namespace thunderbird
