// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Connection manager with automatic retry & reconnect
// ─────────────────────────────────────────────────────────────────────────────
//
// Sits on top of ConnectionStateMachine + ITransport + PacketParser and adds:
//   • Exponential-backoff retry on initial connect failure.
//   • Automatic reconnection when the heartbeat monitor detects link loss.
//   • A single I/O thread that reads from the transport and feeds the parser.
//   • Clean shutdown semantics (stop I/O thread before destroying transport).
//
// Design choices:
//   • **Exponential backoff with jitter**: prevents thundering-herd if many
//     hosts start simultaneously.  Caps at `max_retry_delay_ms`.
//   • **Separate I/O thread**: decouples transport blocking from the user's
//     application thread.  Sensor callbacks are invoked on this I/O thread.
//   • **Reconnect vs. error escalation**: by default the manager retries
//     `max_reconnect_attempts` times.  If exhausted it surfaces an error via
//     the state callback and stops.  The user can call connect() again later.
//   • **Thread-safe public API**: all public methods are safe to call from
//     any thread.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/connection.h"
#include "thunderbird/packet_parser.h"
#include "thunderbird/transport.h"
#include "thunderbird/types.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>

namespace thunderbird {

// ─── Configuration ──────────────────────────────────────────────────────────

struct RetryConfig {
    /// Initial delay before the first retry (ms).
    uint32_t initial_retry_delay_ms   = 500;

    /// Exponential multiplier per successive failure.
    double   backoff_multiplier       = 2.0;

    /// Hard cap on the retry delay (ms).
    uint32_t max_retry_delay_ms       = 30'000;

    /// Maximum number of connect attempts before giving up (0 = infinite).
    uint32_t max_connect_attempts     = 5;

    /// Maximum number of automatic reconnection cycles after link loss (0 = infinite).
    uint32_t max_reconnect_attempts   = 10;

    /// Jitter factor [0, 1].  Adds up to this fraction of the computed delay
    /// as random noise to prevent synchronised retry storms.
    double   jitter_factor            = 0.25;
};

// ─── Event types exposed to the user ────────────────────────────────────────

enum class ConnectionEvent : uint8_t {
    Connected,
    Disconnected,
    Reconnecting,
    ReconnectFailed,     // exhausted all reconnect attempts
    StreamStarted,
    StreamStopped,
    HeartbeatTimeout,
    ProtocolError,
};

inline constexpr const char* connection_event_name(ConnectionEvent e) {
    switch (e) {
        case ConnectionEvent::Connected:        return "Connected";
        case ConnectionEvent::Disconnected:     return "Disconnected";
        case ConnectionEvent::Reconnecting:     return "Reconnecting";
        case ConnectionEvent::ReconnectFailed:  return "ReconnectFailed";
        case ConnectionEvent::StreamStarted:    return "StreamStarted";
        case ConnectionEvent::StreamStopped:    return "StreamStopped";
        case ConnectionEvent::HeartbeatTimeout: return "HeartbeatTimeout";
        case ConnectionEvent::ProtocolError:    return "ProtocolError";
    }
    return "Unknown";
}

using ConnectionEventCallback =
    std::function<void(ConnectionEvent event, const std::string& detail)>;

// ─────────────────────────────────────────────────────────────────────────────

class ConnectionManager {
public:
    explicit ConnectionManager(std::unique_ptr<ITransport> transport,
                               ConnectionConfig conn_cfg = {},
                               RetryConfig      retry_cfg = {})
        : transport_(std::move(transport))
        , conn_cfg_(std::move(conn_cfg))
        , retry_cfg_(std::move(retry_cfg))
        , state_machine_(conn_cfg_)
    {
        // Forward state-machine transitions into our reconnect logic.
        state_machine_.on_state_change(
            [this](ConnectionState old_s, ConnectionState new_s,
                   const std::string& msg) {
                handle_state_change(old_s, new_s, msg);
            });
    }

    ~ConnectionManager() { disconnect(); }

    // Non-copyable
    ConnectionManager(const ConnectionManager&) = delete;
    ConnectionManager& operator=(const ConnectionManager&) = delete;

    // ── Configuration ───────────────────────────────────────────────────────

    void on_event(ConnectionEventCallback cb) {
        std::lock_guard lk(mu_);
        event_cb_ = std::move(cb);
    }

    /// Access the parser to register sensor callbacks.
    PacketParser& parser() { return parser_; }

    // ── Lifecycle ───────────────────────────────────────────────────────────

    /// Blocking connect with retry.  Returns OK on success, or the last
    /// error after exhausting all attempts.
    Status connect(const std::string& uri) {
        uri_ = uri;
        return connect_with_retry();
    }

    /// Start streaming: sends StartStream to the device and begins the
    /// I/O reader thread.
    Status start_streaming() {
        Status s = state_machine_.request_start_stream();
        if (s != Status::OK) return s;

        start_io_thread();
        emit(ConnectionEvent::StreamStarted, "");
        return Status::OK;
    }

    Status stop_streaming() {
        stop_io_thread();
        Status s = state_machine_.request_stop_stream();
        emit(ConnectionEvent::StreamStopped, "");
        return s;
    }

    void disconnect() {
        should_reconnect_ = false;
        stop_io_thread();
        state_machine_.disconnect();
        emit(ConnectionEvent::Disconnected, "User disconnect");
    }

    ConnectionState state() const { return state_machine_.state(); }

    DeviceInfo device_info() const { return state_machine_.device_info(); }

    const ParserStats& parser_stats() const { return parser_.stats(); }

private:
    // ── Connect with exponential backoff ────────────────────────────────────

    Status connect_with_retry() {
        std::mt19937 rng(std::random_device{}());

        uint32_t attempt = 0;
        double delay_ms  = retry_cfg_.initial_retry_delay_ms;

        while (true) {
            ++attempt;
            Status s = state_machine_.connect(*transport_, uri_);
            if (s == Status::OK) {
                should_reconnect_ = true;
                emit(ConnectionEvent::Connected,
                     "Connected after " + std::to_string(attempt) + " attempt(s)");
                return Status::OK;
            }

            // Check attempt limit.
            if (retry_cfg_.max_connect_attempts > 0 &&
                attempt >= retry_cfg_.max_connect_attempts) {
                emit(ConnectionEvent::ReconnectFailed,
                     "Exhausted " + std::to_string(attempt) + " connect attempts");
                return s;
            }

            // Compute jittered delay.
            double jitter = 0;
            if (retry_cfg_.jitter_factor > 0) {
                std::uniform_real_distribution<double> dist(
                    0, delay_ms * retry_cfg_.jitter_factor);
                jitter = dist(rng);
            }

            uint32_t sleep_ms = static_cast<uint32_t>(delay_ms + jitter);
            emit(ConnectionEvent::Reconnecting,
                 "Retry #" + std::to_string(attempt) +
                 " in " + std::to_string(sleep_ms) + " ms");

            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));

            // Backoff.
            delay_ms = std::min(
                delay_ms * retry_cfg_.backoff_multiplier,
                static_cast<double>(retry_cfg_.max_retry_delay_ms));

            // Reset transport for the next attempt.
            transport_->close();
        }
    }

    // ── I/O reader thread ───────────────────────────────────────────────────

    void start_io_thread() {
        if (io_running_.exchange(true)) return;
        io_thread_ = std::thread([this] { io_loop(); });
    }

    void stop_io_thread() {
        io_running_ = false;
        if (io_thread_.joinable()) io_thread_.join();
    }

    void io_loop() {
        constexpr size_t kBufSize = 64 * 1024; // 64 KB read buffer
        auto buf = std::make_unique<uint8_t[]>(kBufSize);

        while (io_running_) {
            if (!transport_->is_open()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            size_t n = transport_->read(buf.get(), kBufSize, /*timeout_ms=*/100);
            if (n > 0) {
                parser_.feed(buf.get(), n);
            }
        }
    }

    // ── State-change handler (reconnect logic) ──────────────────────────────

    void handle_state_change(ConnectionState /*old_s*/, ConnectionState new_s,
                             const std::string& msg) {
        if (new_s == ConnectionState::Error && should_reconnect_) {
            emit(ConnectionEvent::HeartbeatTimeout, msg);

            // Attempt reconnection on a detached thread to avoid blocking
            // the heartbeat timer thread.
            std::thread([this] { attempt_reconnect(); }).detach();
        }
    }

    void attempt_reconnect() {
        uint32_t attempt = 0;
        double delay_ms = retry_cfg_.initial_retry_delay_ms;
        std::mt19937 rng(std::random_device{}());

        while (should_reconnect_) {
            ++attempt;
            if (retry_cfg_.max_reconnect_attempts > 0 &&
                attempt > retry_cfg_.max_reconnect_attempts) {
                emit(ConnectionEvent::ReconnectFailed,
                     "Exhausted " + std::to_string(attempt - 1) + " reconnect attempts");
                return;
            }

            // Close stale transport and retry.
            transport_->close();
            parser_.reset();

            emit(ConnectionEvent::Reconnecting,
                 "Reconnect attempt " + std::to_string(attempt));

            Status s = state_machine_.connect(*transport_, uri_);
            if (s == Status::OK) {
                emit(ConnectionEvent::Connected, "Reconnected");
                // Re-start streaming if we were streaming before.
                state_machine_.request_start_stream();
                start_io_thread();
                return;
            }

            double jitter = 0;
            if (retry_cfg_.jitter_factor > 0) {
                std::uniform_real_distribution<double> dist(
                    0, delay_ms * retry_cfg_.jitter_factor);
                jitter = dist(rng);
            }
            std::this_thread::sleep_for(
                std::chrono::milliseconds(static_cast<uint32_t>(delay_ms + jitter)));

            delay_ms = std::min(delay_ms * retry_cfg_.backoff_multiplier,
                                static_cast<double>(retry_cfg_.max_retry_delay_ms));
        }
    }

    // ── Event emission ──────────────────────────────────────────────────────

    void emit(ConnectionEvent evt, const std::string& detail) {
        ConnectionEventCallback cb;
        {
            std::lock_guard lk(mu_);
            cb = event_cb_;
        }
        if (cb) cb(evt, detail);
    }

    // ── Members ─────────────────────────────────────────────────────────────

    std::unique_ptr<ITransport>  transport_;
    ConnectionConfig             conn_cfg_;
    RetryConfig                  retry_cfg_;
    ConnectionStateMachine       state_machine_;
    PacketParser                 parser_;
    std::string                  uri_;

    std::mutex                   mu_;
    ConnectionEventCallback      event_cb_;

    std::atomic<bool>            io_running_{false};
    std::thread                  io_thread_;
    std::atomic<bool>            should_reconnect_{false};
};

} // namespace thunderbird
