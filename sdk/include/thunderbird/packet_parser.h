// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Incremental packet parser
// ─────────────────────────────────────────────────────────────────────────────
//
// Converts a raw byte stream (from any ITransport) into typed SDK objects
// (LidarFrame, ImuSample, CameraFrame) plus control messages.
//
// Design rationale:
//   • **Incremental / streaming**: bytes can arrive in arbitrary chunks
//     (partial headers, split payloads).  The parser is a state machine
//     that accumulates bytes until a full packet is assembled.
//   • **Zero-copy where possible**: for large payloads (camera), the parser
//     writes directly into the destination vector rather than making an
//     intermediate copy.
//   • **CRC validation**: every assembled packet is CRC-checked before
//     dispatch.  On CRC failure the parser re-syncs by scanning for the
//     next magic word.
//   • **Callbacks by packet type**: users register per-type handlers.
//     This keeps the parser reusable across both the control path and
//     the sensor data path.
//
// Usage:
//   PacketParser parser;
//   parser.on_lidar([](auto frame) { ... });
//   parser.on_imu([](auto sample) { ... });
//   parser.on_camera([](auto frame) { ... });
//   parser.on_control([](PacketType t, auto hdr, span<uint8_t> payload) { ... });
//
//   // In your I/O loop:
//   size_t n = transport.read(buf, sizeof(buf), timeout);
//   parser.feed(buf, n);
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/protocol.h"
#include "thunderbird/types.h"

#include <cstring>
#include <functional>
#include <vector>
#include <memory>

namespace thunderbird {

// ─── Control packet callback ────────────────────────────────────────────────

using ControlCallback = std::function<void(
    protocol::PacketType           type,
    const protocol::PacketHeader&  header,
    const uint8_t*                 payload,
    size_t                         payload_len)>;

// ─── Parser statistics ──────────────────────────────────────────────────────

struct ParserStats {
    uint64_t packets_parsed{0};
    uint64_t crc_errors{0};
    uint64_t resync_count{0};       // number of times we had to scan for magic
    uint64_t bytes_processed{0};
};

// ─────────────────────────────────────────────────────────────────────────────

class PacketParser {
public:
    PacketParser() { buf_.reserve(64 * 1024); } // pre-allocate 64 KB

    // ── Register callbacks ──────────────────────────────────────────────────

    void on_lidar(LidarCallback cb)       { lidar_cb_   = std::move(cb); }
    void on_imu(ImuCallback cb)           { imu_cb_     = std::move(cb); }
    void on_camera(CameraCallback cb)     { camera_cb_  = std::move(cb); }
    void on_control(ControlCallback cb)   { control_cb_ = std::move(cb); }

    // ── Feed raw bytes ──────────────────────────────────────────────────────

    /// Append raw bytes from the transport and parse as many complete packets
    /// as possible.  Safe to call with partial data; leftover bytes are
    /// retained for the next call.
    void feed(const uint8_t* data, size_t len) {
        buf_.insert(buf_.end(), data, data + len);
        stats_.bytes_processed += len;

        // Process all complete packets in the buffer.
        while (try_parse_one()) { /* keep going */ }
    }

    /// Reset internal buffer (e.g. after a reconnect).
    void reset() {
        buf_.clear();
    }

    const ParserStats& stats() const { return stats_; }

private:
    // ── Core parse loop ─────────────────────────────────────────────────────

    /// Try to extract one packet from the front of buf_.
    /// Returns true if a packet was consumed (buffer was shrunk).
    bool try_parse_one() {
        // Need at least a header to inspect.
        if (buf_.size() < protocol::kHeaderSize)
            return false;

        // ── Step 1: locate magic ────────────────────────────────────────
        size_t offset = find_magic();
        if (offset == std::string::npos)  {
            // No magic found at all — discard everything except last byte
            // (which could be the first byte of a split magic).
            if (buf_.size() > 1) {
                buf_.erase(buf_.begin(), buf_.end() - 1);
            }
            return false;
        }

        if (offset > 0) {
            // Discard bytes before the magic (junk / desync data).
            buf_.erase(buf_.begin(), buf_.begin() + static_cast<ptrdiff_t>(offset));
            ++stats_.resync_count;
        }

        if (buf_.size() < protocol::kHeaderSize)
            return false;

        // ── Step 2: read header ─────────────────────────────────────────
        protocol::PacketHeader hdr;
        std::memcpy(&hdr, buf_.data(), protocol::kHeaderSize);

        // Validate fields before trusting payload_length.
        if (hdr.version != protocol::kProtocolVersion) {
            // Unknown version — skip these two magic bytes and try again.
            buf_.erase(buf_.begin(), buf_.begin() + 2);
            ++stats_.resync_count;
            return true; // retry
        }

        if (hdr.payload_length > protocol::kMaxPayloadBytes) {
            // Corrupt length — skip magic and resync.
            buf_.erase(buf_.begin(), buf_.begin() + 2);
            ++stats_.resync_count;
            return true;
        }

        // ── Step 3: wait for complete packet ────────────────────────────
        size_t total = protocol::kHeaderSize + hdr.payload_length + protocol::kCrcSize;
        if (buf_.size() < total)
            return false; // need more bytes

        // ── Step 4: CRC check ───────────────────────────────────────────
        if (!protocol::Crc32::validate(buf_.data(), total)) {
            // CRC failure — skip magic and resync.
            ++stats_.crc_errors;
            buf_.erase(buf_.begin(), buf_.begin() + 2);
            ++stats_.resync_count;
            return true;
        }

        // ── Step 5: dispatch ────────────────────────────────────────────
        const uint8_t* payload_ptr = buf_.data() + protocol::kHeaderSize;
        uint32_t payload_len = hdr.payload_length;

        dispatch(hdr, payload_ptr, payload_len);
        ++stats_.packets_parsed;

        // Remove consumed bytes.
        buf_.erase(buf_.begin(), buf_.begin() + static_cast<ptrdiff_t>(total));
        return true;
    }

    // ── Magic scan ──────────────────────────────────────────────────────────

    /// Find the first occurrence of the two-byte magic in buf_.
    /// Returns offset, or std::string::npos if not found.
    size_t find_magic() const {
        if (buf_.size() < 2) return std::string::npos;
        const uint8_t lo = static_cast<uint8_t>(protocol::kMagic & 0xFF);
        const uint8_t hi = static_cast<uint8_t>(protocol::kMagic >> 8);
        for (size_t i = 0; i + 1 < buf_.size(); ++i) {
            if (buf_[i] == lo && buf_[i + 1] == hi)
                return i;
        }
        return std::string::npos;
    }

    // ── Dispatch by packet type ─────────────────────────────────────────────

    void dispatch(const protocol::PacketHeader& hdr,
                  const uint8_t* payload, uint32_t payload_len)
    {
        Timestamp host_ts = Timestamp::now();
        Timestamp hw_ts{hdr.hw_timestamp_ns};
        auto ptype = static_cast<protocol::PacketType>(hdr.payload_type);

        switch (ptype) {

        case protocol::PacketType::LidarScan:
            dispatch_lidar(hdr, hw_ts, host_ts, payload, payload_len);
            break;

        case protocol::PacketType::ImuSample:
            dispatch_imu(hdr, hw_ts, host_ts, payload, payload_len);
            break;

        case protocol::PacketType::CameraFrame:
            dispatch_camera(hdr, hw_ts, host_ts, payload, payload_len);
            break;

        default:
            // Control / handshake / heartbeat / unknown
            if (control_cb_)
                control_cb_(ptype, hdr, payload, payload_len);
            break;
        }
    }

    // ── Sensor dispatch helpers ─────────────────────────────────────────────

    void dispatch_lidar(const protocol::PacketHeader& hdr,
                        Timestamp hw_ts, Timestamp host_ts,
                        const uint8_t* payload, uint32_t payload_len)
    {
        if (!lidar_cb_) return;
        if (payload_len < sizeof(protocol::LidarSubHeader)) return;

        protocol::LidarSubHeader sub;
        std::memcpy(&sub, payload, sizeof(sub));

        const size_t points_bytes = payload_len - sizeof(sub);
        const size_t point_count  = points_bytes / sizeof(protocol::LidarWirePoint);
        // Clamp to declared count to avoid over-read.
        const size_t n = std::min(static_cast<size_t>(sub.num_points), point_count);

        auto frame = std::make_shared<LidarFrame>();
        frame->timestamp       = hw_ts;
        frame->host_timestamp  = host_ts;
        frame->sequence_number = hdr.sequence;
        frame->points.resize(n);

        const auto* wire_pts = reinterpret_cast<const protocol::LidarWirePoint*>(
            payload + sizeof(sub));

        for (size_t i = 0; i < n; ++i) {
            frame->points[i].x         = wire_pts[i].x;
            frame->points[i].y         = wire_pts[i].y;
            frame->points[i].z         = wire_pts[i].z;
            frame->points[i].intensity = static_cast<float>(wire_pts[i].intensity);
            frame->points[i].ring      = wire_pts[i].ring;
        }

        lidar_cb_(std::move(frame));
    }

    void dispatch_imu(const protocol::PacketHeader& hdr,
                      Timestamp hw_ts, Timestamp host_ts,
                      const uint8_t* payload, uint32_t payload_len)
    {
        if (!imu_cb_) return;
        if (payload_len < sizeof(protocol::ImuWirePayload)) return;

        protocol::ImuWirePayload wire;
        std::memcpy(&wire, payload, sizeof(wire));

        auto sample = std::make_shared<ImuSample>();
        sample->timestamp      = hw_ts;
        sample->host_timestamp = host_ts;
        sample->accel          = {wire.accel_x, wire.accel_y, wire.accel_z};
        sample->gyro           = {wire.gyro_x,  wire.gyro_y,  wire.gyro_z};
        sample->temperature    = wire.temperature;

        (void)hdr; // sequence available if needed
        imu_cb_(std::move(sample));
    }

    void dispatch_camera(const protocol::PacketHeader& hdr,
                         Timestamp hw_ts, Timestamp host_ts,
                         const uint8_t* payload, uint32_t payload_len)
    {
        if (!camera_cb_) return;
        if (payload_len < sizeof(protocol::CameraSubHeader)) return;

        protocol::CameraSubHeader sub;
        std::memcpy(&sub, payload, sizeof(sub));

        const uint8_t* pixel_data = payload + sizeof(sub);
        const size_t pixel_bytes  = payload_len - sizeof(sub);
        const size_t actual_bytes = std::min(static_cast<size_t>(sub.data_length),
                                             pixel_bytes);

        auto frame = std::make_shared<CameraFrame>();
        frame->timestamp       = hw_ts;
        frame->host_timestamp  = host_ts;
        frame->sequence_number = hdr.sequence;
        frame->width           = sub.width;
        frame->height          = sub.height;
        frame->format          = static_cast<PixelFormat>(sub.pixel_format);

        // Compute stride based on format.
        uint32_t bpp = (frame->format == PixelFormat::Mono8) ? 1u : 3u;
        frame->stride = sub.width * bpp;

        frame->data.assign(pixel_data, pixel_data + actual_bytes);

        camera_cb_(std::move(frame));
    }

    // ── State ───────────────────────────────────────────────────────────────

    std::vector<uint8_t> buf_;       // accumulation buffer
    ParserStats          stats_{};

    LidarCallback   lidar_cb_;
    ImuCallback     imu_cb_;
    CameraCallback  camera_cb_;
    ControlCallback control_cb_;
};

} // namespace thunderbird
