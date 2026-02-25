// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Connection state machine
// ─────────────────────────────────────────────────────────────────────────────
//
// Models the full lifecycle of a device connection:
//
//   Disconnected ──connect()──▶ Connecting ──▶ Handshake ──▶ Connected
//        ▲                         │                │            │
//        │         timeout/error ──┘    reject ─────┘            │
//        │                                                       │
//        └──────────────── disconnect() ◀────────────────────────┘
//
// Design choices:
//   • Enum-based state with atomic transitions — simple, debuggable.
//   • The state machine itself is transport-agnostic; it delegates I/O
//     to an ITransport* and parsing to the PacketParser.
//   • Observer pattern: register a callback for state changes so the
//     DeviceManager can react (e.g. auto-restart streaming after reconnect).
//   • Heartbeat monitoring runs on a background thread; if N consecutive
//     heartbeats are missed the connection transitions to Disconnected.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/types.h"
#include "thunderbird/transport.h"
#include "thunderbird/protocol.h"

#include <atomic>
#include <chrono>
#include <cstring>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace thunderbird {

// ─── Connection states ──────────────────────────────────────────────────────

enum class ConnectionState : uint8_t {
    Disconnected,
    Connecting,      // transport open in progress
    Handshake,       // waiting for HandshakeAck from device
    Connected,       // ready — can issue StartStream
    Streaming,       // actively receiving sensor data
    Error,           // unrecoverable error (user must call disconnect())
};

inline constexpr const char* connection_state_name(ConnectionState s) {
    switch (s) {
        case ConnectionState::Disconnected: return "Disconnected";
        case ConnectionState::Connecting:   return "Connecting";
        case ConnectionState::Handshake:    return "Handshake";
        case ConnectionState::Connected:    return "Connected";
        case ConnectionState::Streaming:    return "Streaming";
        case ConnectionState::Error:        return "Error";
    }
    return "Unknown";
}

// ─── Configuration ──────────────────────────────────────────────────────────

struct ConnectionConfig {
    /// Maximum time to wait for the transport open() call (ms).
    uint32_t connect_timeout_ms    = 3000;

    /// Maximum time to wait for a HandshakeAck after sending Handshake (ms).
    uint32_t handshake_timeout_ms  = 2000;

    /// Interval between host → device heartbeats (ms).
    uint32_t heartbeat_interval_ms = 1000;

    /// Number of missed heartbeat replies before declaring link dead.
    uint32_t heartbeat_miss_limit  = 3;

    /// Client identifier embedded in the Handshake payload.
    std::string client_id          = "thunderbird-sdk";
};

// ─── Observer callback type ─────────────────────────────────────────────────

/// Fired on every state transition.  `old_state` → `new_state`.
/// The optional `message` carries human-readable context (e.g. error text).
using ConnectionStateCallback =
    std::function<void(ConnectionState old_state,
                       ConnectionState new_state,
                       const std::string& message)>;

// ─── ConnectionStateMachine ─────────────────────────────────────────────────

class ConnectionStateMachine {
public:
    explicit ConnectionStateMachine(ConnectionConfig config = {})
        : config_(std::move(config)) {}

    ~ConnectionStateMachine() { stop_heartbeat(); }

    // Non-copyable, non-movable (owns a thread).
    ConnectionStateMachine(const ConnectionStateMachine&) = delete;
    ConnectionStateMachine& operator=(const ConnectionStateMachine&) = delete;

    // ── Queries ─────────────────────────────────────────────────────────────

    ConnectionState state() const { return state_.load(std::memory_order_acquire); }

    // ── Observer ────────────────────────────────────────────────────────────

    void on_state_change(ConnectionStateCallback cb) {
        std::lock_guard lk(mu_);
        state_cb_ = std::move(cb);
    }

    // ── Actions ─────────────────────────────────────────────────────────────

    /// Attempt to open the transport and perform the handshake.
    /// Blocks until Connected or returns an error Status.
    Status connect(ITransport& transport, const std::string& uri) {
        if (state() != ConnectionState::Disconnected &&
            state() != ConnectionState::Error)
            return Status::AlreadyStreaming; // already connected or connecting

        transport_ = &transport;

        // ── Phase 1: open transport ─────────────────────────────────────
        transition(ConnectionState::Connecting, "Opening transport: " + uri);

        Status s = transport_->open(uri);
        if (s != Status::OK) {
            transition(ConnectionState::Error, "Transport open failed");
            return s;
        }

        // ── Phase 2: send Handshake ─────────────────────────────────────
        transition(ConnectionState::Handshake, "Sending handshake");

        protocol::HandshakePayload hs{};
        hs.protocol_version = protocol::kProtocolVersion;
        std::strncpy(hs.client_id, config_.client_id.c_str(),
                     sizeof(hs.client_id) - 1);

        uint8_t pkt_buf[256];
        size_t pkt_len = protocol::build_packet(
            pkt_buf, sizeof(pkt_buf),
            protocol::PacketType::Handshake, 0, 0,
            reinterpret_cast<const uint8_t*>(&hs), sizeof(hs));

        if (transport_->write(pkt_buf, pkt_len) != pkt_len) {
            transition(ConnectionState::Error, "Failed to send handshake");
            return Status::TransportError;
        }

        // ── Phase 3: wait for HandshakeAck ──────────────────────────────
        uint8_t recv_buf[512];
        size_t n = transport_->read(recv_buf, sizeof(recv_buf),
                                    config_.handshake_timeout_ms);

        if (n < protocol::kHeaderSize + protocol::kCrcSize) {
            transition(ConnectionState::Error, "Handshake timeout / short read");
            transport_->close();
            return Status::Timeout;
        }

        if (!protocol::Crc32::validate(recv_buf, n)) {
            transition(ConnectionState::Error, "Handshake CRC mismatch");
            transport_->close();
            return Status::TransportError;
        }

        protocol::PacketHeader hdr;
        std::memcpy(&hdr, recv_buf, sizeof(hdr));

        if (static_cast<protocol::PacketType>(hdr.payload_type) !=
            protocol::PacketType::HandshakeAck) {
            transition(ConnectionState::Error, "Unexpected packet during handshake");
            transport_->close();
            return Status::TransportError;
        }

        // Extract device info from the ack payload.
        if (hdr.payload_length >= sizeof(protocol::HandshakeAckPayload)) {
            protocol::HandshakeAckPayload ack;
            std::memcpy(&ack, recv_buf + protocol::kHeaderSize, sizeof(ack));
            if (!ack.accepted) {
                transition(ConnectionState::Error, "Device rejected handshake");
                transport_->close();
                return Status::TransportError;
            }
            std::lock_guard lk(mu_);
            dev_info_.serial_number    = ack.serial_number;
            dev_info_.firmware_version = ack.firmware_version;
            dev_info_.model_name       = ack.model_name;
        }

        // ── Phase 4: connected ──────────────────────────────────────────
        transition(ConnectionState::Connected, "Handshake complete");
        start_heartbeat();
        return Status::OK;
    }

    /// Send StartStream command to the device.
    Status request_start_stream() {
        if (state() != ConnectionState::Connected)
            return Status::NotConnected;

        uint8_t pkt[64];
        size_t len = protocol::build_packet(
            pkt, sizeof(pkt),
            protocol::PacketType::StartStream, 0, 0, nullptr, 0);
        if (transport_->write(pkt, len) != len)
            return Status::TransportError;

        transition(ConnectionState::Streaming, "Stream started");
        return Status::OK;
    }

    /// Send StopStream command to the device.
    Status request_stop_stream() {
        if (state() != ConnectionState::Streaming)
            return Status::OK; // idempotent

        uint8_t pkt[64];
        size_t len = protocol::build_packet(
            pkt, sizeof(pkt),
            protocol::PacketType::StopStream, 0, 0, nullptr, 0);
        transport_->write(pkt, len); // best-effort

        transition(ConnectionState::Connected, "Stream stopped");
        return Status::OK;
    }

    /// Gracefully close the connection.
    void disconnect() {
        stop_heartbeat();
        if (transport_ && transport_->is_open())
            transport_->close();
        transition(ConnectionState::Disconnected, "User requested disconnect");
        transport_ = nullptr;
    }

    /// Called by external code when a heartbeat reply is received.
    void notify_heartbeat_received() {
        missed_heartbeats_.store(0, std::memory_order_release);
    }

    /// Retrieve device info obtained during handshake.
    DeviceInfo device_info() const {
        std::lock_guard lk(mu_);
        return dev_info_;
    }

private:
    void transition(ConnectionState new_state, const std::string& msg) {
        ConnectionState old = state_.exchange(new_state, std::memory_order_acq_rel);
        ConnectionStateCallback cb;
        {
            std::lock_guard lk(mu_);
            cb = state_cb_;
        }
        if (cb) cb(old, new_state, msg);
    }

    // ── Heartbeat thread ────────────────────────────────────────────────────

    void start_heartbeat() {
        hb_running_ = true;
        hb_thread_ = std::thread([this] { heartbeat_loop(); });
    }

    void stop_heartbeat() {
        hb_running_ = false;
        if (hb_thread_.joinable()) hb_thread_.join();
    }

    void heartbeat_loop() {
        while (hb_running_) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(config_.heartbeat_interval_ms));
            if (!hb_running_) break;

            auto current = state();
            if (current != ConnectionState::Connected &&
                current != ConnectionState::Streaming)
                continue;

            // Build and send heartbeat
            protocol::HeartbeatPayload hb{};
            hb.sender_timestamp_ns = Timestamp::now().nanoseconds;
            hb.uptime_seconds      = 0;
            hb.error_flags         = 0;

            uint8_t pkt[64];
            size_t len = protocol::build_packet(
                pkt, sizeof(pkt),
                protocol::PacketType::Heartbeat, 0,
                hb.sender_timestamp_ns,
                reinterpret_cast<const uint8_t*>(&hb), sizeof(hb));

            if (transport_ && transport_->is_open()) {
                transport_->write(pkt, len);
            }

            // Check for missed replies
            uint32_t missed = missed_heartbeats_.fetch_add(1, std::memory_order_acq_rel) + 1;
            if (missed >= config_.heartbeat_miss_limit) {
                transition(ConnectionState::Error,
                           "Heartbeat timeout (" + std::to_string(missed) + " missed)");
                hb_running_ = false;
            }
        }
    }

    ConnectionConfig             config_;
    std::atomic<ConnectionState> state_{ConnectionState::Disconnected};
    mutable std::mutex           mu_;
    ConnectionStateCallback      state_cb_;
    DeviceInfo                   dev_info_;
    ITransport*                  transport_{nullptr};

    // Heartbeat
    std::atomic<bool>            hb_running_{false};
    std::atomic<uint32_t>        missed_heartbeats_{0};
    std::thread                  hb_thread_;
};

} // namespace thunderbird
