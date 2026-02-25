// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Wire protocol definition
// ─────────────────────────────────────────────────────────────────────────────
//
// Binary packet format for device ↔ host communication over any transport.
//
// Design rationale:
//   • Fixed-size header allows O(1) framing without stream-level escaping.
//   • CRC-32 on the full packet ensures integrity over noisy USB/Ethernet.
//   • Hardware timestamp embedded in header — no host-side inference needed.
//   • Payload is sensor-specific; header.payload_type selects the parser.
//   • Sequence numbers per-channel detect drops without transport-level ACKs.
//   • Magic bytes + version field let us evolve the protocol while staying
//     backwards-compatible (older host can reject newer packets gracefully).
//
// Packet layout (little-endian):
//
//   Offset  Size   Field
//   ──────  ─────  ──────────────────────────────
//     0      2     magic            0xBEEF
//     2      1     version          protocol version (currently 1)
//     3      1     payload_type     PacketType enum
//     4      4     sequence         per-channel monotonic counter
//     8      8     hw_timestamp_ns  device-side nanosecond timestamp
//    16      4     payload_length   byte count of payload (excl. header+crc)
//    20      N     payload          sensor-specific bytes
//   20+N     4     crc32            CRC-32 over bytes [0 .. 20+N-1]
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <array>
#include <span>

namespace thunderbird::protocol {

// ─── Constants ──────────────────────────────────────────────────────────────

/// Two-byte magic that starts every packet (little-endian: 0xEF, 0xBE on wire).
inline constexpr uint16_t kMagic           = 0xBEEF;

/// Current wire-protocol version.
inline constexpr uint8_t  kProtocolVersion = 1;

/// Fixed header size in bytes.
inline constexpr size_t   kHeaderSize      = 20;

/// CRC-32 footer size.
inline constexpr size_t   kCrcSize         = 4;

/// Maximum payload we'll ever accept (16 MiB — covers a 4K camera frame).
inline constexpr uint32_t kMaxPayloadBytes = 16u * 1024u * 1024u;

// ─── Packet types ───────────────────────────────────────────────────────────

/// Identifies the sensor / control channel that produced the payload.
enum class PacketType : uint8_t {
    // ── Sensor data (device → host) ─────────────────────────────────────
    LidarScan     = 0x01,   // one full or partial LiDAR sweep
    ImuSample     = 0x02,   // single IMU measurement
    CameraFrame   = 0x03,   // one image (may be chunked for large frames)

    // ── Control / handshake (bidirectional) ─────────────────────────────
    Handshake     = 0x10,   // initial connection negotiation
    HandshakeAck  = 0x11,   // device confirms connection
    StartStream   = 0x12,   // host → device: begin streaming
    StopStream    = 0x13,   // host → device: stop streaming
    Heartbeat     = 0x14,   // keepalive (both directions)
    DeviceInfo    = 0x15,   // device → host: serial, FW, model

    // ── Error ───────────────────────────────────────────────────────────
    Error         = 0xFF,
};

inline constexpr const char* packet_type_name(PacketType t) {
    switch (t) {
        case PacketType::LidarScan:    return "LidarScan";
        case PacketType::ImuSample:    return "ImuSample";
        case PacketType::CameraFrame:  return "CameraFrame";
        case PacketType::Handshake:    return "Handshake";
        case PacketType::HandshakeAck: return "HandshakeAck";
        case PacketType::StartStream:  return "StartStream";
        case PacketType::StopStream:   return "StopStream";
        case PacketType::Heartbeat:    return "Heartbeat";
        case PacketType::DeviceInfo:   return "DeviceInfo";
        case PacketType::Error:        return "Error";
    }
    return "Unknown";
}

// ─── On-wire header ─────────────────────────────────────────────────────────

/// Packed header laid out exactly as it appears on the wire.
/// All multi-byte fields are little-endian.
#pragma pack(push, 1)
struct PacketHeader {
    uint16_t magic{kMagic};
    uint8_t  version{kProtocolVersion};
    uint8_t  payload_type{0};            // PacketType
    uint32_t sequence{0};
    int64_t  hw_timestamp_ns{0};
    uint32_t payload_length{0};
};
static_assert(sizeof(PacketHeader) == kHeaderSize,
              "PacketHeader must be exactly 20 bytes");
#pragma pack(pop)

// ─── LiDAR payload sub-header ──────────────────────────────────────────────

/// Immediately follows PacketHeader when packet_type == LidarScan.
/// The actual point data (array of LidarWirePoint) follows this sub-header.
#pragma pack(push, 1)
struct LidarSubHeader {
    uint32_t num_points{0};      // number of LidarWirePoint that follow
    float    azimuth_start{0};   // degrees — start of this scan segment
    float    azimuth_end{0};     // degrees — end of this scan segment
};

/// Single LiDAR point on the wire — tightly packed for bandwidth.
struct LidarWirePoint {
    float   x, y, z;            // metres
    uint8_t intensity;           // 0–255
    uint8_t ring;                // channel id
    uint8_t _pad[2]{};           // align to 16 bytes
};
static_assert(sizeof(LidarWirePoint) == 16);
#pragma pack(pop)

// ─── IMU payload ────────────────────────────────────────────────────────────

#pragma pack(push, 1)
struct ImuWirePayload {
    float accel_x, accel_y, accel_z;   // m/s²
    float gyro_x,  gyro_y,  gyro_z;   // rad/s
    float temperature;                  // °C
};
static_assert(sizeof(ImuWirePayload) == 28);
#pragma pack(pop)

// ─── Camera payload sub-header ──────────────────────────────────────────────

#pragma pack(push, 1)
struct CameraSubHeader {
    uint32_t width;
    uint32_t height;
    uint8_t  pixel_format;       // maps to PixelFormat enum
    uint8_t  _pad[3]{};
    uint32_t data_length;        // bytes of raw pixel data that follow
};
static_assert(sizeof(CameraSubHeader) == 16);
#pragma pack(pop)

// ─── Handshake payload ──────────────────────────────────────────────────────

#pragma pack(push, 1)
struct HandshakePayload {
    uint8_t  protocol_version;
    uint8_t  _pad[3]{};
    char     client_id[32]{};            // null-terminated string
};

struct HandshakeAckPayload {
    uint8_t  accepted;                   // 1 = OK, 0 = rejected
    uint8_t  device_protocol_version;
    uint8_t  _pad[2]{};
    char     serial_number[32]{};
    char     firmware_version[32]{};
    char     model_name[32]{};
};
#pragma pack(pop)

// ─── Heartbeat payload ──────────────────────────────────────────────────────

#pragma pack(push, 1)
struct HeartbeatPayload {
    int64_t  sender_timestamp_ns;        // for round-trip latency estimation
    uint32_t uptime_seconds;
    uint32_t error_flags;                // bit-field; 0 = healthy
};
#pragma pack(pop)

// ─── CRC-32 (IEEE 802.3) ───────────────────────────────────────────────────

/// Simple table-driven CRC-32 implementation.
/// We compute over the full packet (header + payload) and append 4 bytes.
namespace detail {
/// Build the CRC-32 lookup table at compile time.
inline constexpr std::array<uint32_t, 256> build_crc32_table() {
    std::array<uint32_t, 256> t{};
    for (uint32_t i = 0; i < 256; ++i) {
        uint32_t c = i;
        for (int j = 0; j < 8; ++j)
            c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        t[i] = c;
    }
    return t;
}
inline constexpr std::array<uint32_t, 256> kCrc32Table = build_crc32_table();
} // namespace detail

/// Simple table-driven CRC-32 implementation (IEEE 802.3).
/// We compute over the full packet (header + payload) and append 4 bytes.
class Crc32 {
public:
    /// Compute CRC-32 over a byte range.
    static uint32_t compute(const uint8_t* data, size_t len) {
        uint32_t crc = 0xFFFFFFFFu;
        for (size_t i = 0; i < len; ++i) {
            crc = detail::kCrc32Table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
        }
        return crc ^ 0xFFFFFFFFu;
    }

    /// Validate: recompute CRC and compare against the trailing 4 bytes.
    static bool validate(const uint8_t* packet, size_t total_len) {
        if (total_len < kCrcSize) return false;
        uint32_t stored = 0;
        std::memcpy(&stored, packet + total_len - kCrcSize, kCrcSize);
        uint32_t computed = compute(packet, total_len - kCrcSize);
        return stored == computed;
    }
};

// ─── Packet builder helper ──────────────────────────────────────────────────

/// Serialise a header + payload + CRC into a contiguous buffer for transmission.
/// Returns total packet size (header + payload + crc).
inline size_t build_packet(uint8_t* out, size_t out_capacity,
                           PacketType type, uint32_t seq,
                           int64_t hw_ts_ns,
                           const uint8_t* payload, uint32_t payload_len)
{
    const size_t total = kHeaderSize + payload_len + kCrcSize;
    if (total > out_capacity) return 0;

    PacketHeader hdr;
    hdr.payload_type   = static_cast<uint8_t>(type);
    hdr.sequence       = seq;
    hdr.hw_timestamp_ns = hw_ts_ns;
    hdr.payload_length = payload_len;

    std::memcpy(out, &hdr, kHeaderSize);
    if (payload_len > 0)
        std::memcpy(out + kHeaderSize, payload, payload_len);

    uint32_t crc = Crc32::compute(out, kHeaderSize + payload_len);
    std::memcpy(out + kHeaderSize + payload_len, &crc, kCrcSize);

    return total;
}

} // namespace thunderbird::protocol
