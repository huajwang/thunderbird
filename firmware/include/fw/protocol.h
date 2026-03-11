// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird Firmware — Wire protocol constants & helpers (C port)
// ─────────────────────────────────────────────────────────────────────────────
//
// Mirrors the definitions in sdk/include/thunderbird/protocol.h but in
// plain C for firmware portability.  Kept in sync manually — the wire
// format is the contract between firmware and host SDK.
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// ─── Constants ──────────────────────────────────────────────────────────────

#define FW_PROTO_MAGIC           0xBEEFu
#define FW_PROTO_VERSION         1u
#define FW_PROTO_HEADER_SIZE     20u
#define FW_PROTO_CRC_SIZE        4u
#define FW_PROTO_MAX_PAYLOAD     (16u * 1024u * 1024u)

// ─── Packet types (must match SDK PacketType enum) ──────────────────────────

#define FW_PKT_LIDAR_SCAN       0x01u
#define FW_PKT_IMU_SAMPLE       0x02u
#define FW_PKT_CAMERA_FRAME     0x03u
#define FW_PKT_HANDSHAKE        0x10u
#define FW_PKT_HANDSHAKE_ACK    0x11u
#define FW_PKT_START_STREAM     0x12u
#define FW_PKT_STOP_STREAM      0x13u
#define FW_PKT_HEARTBEAT        0x14u
#define FW_PKT_DEVICE_INFO      0x15u
#define FW_PKT_ERROR            0xFFu

// ─── On-wire header ─────────────────────────────────────────────────────────

#pragma pack(push, 1)
typedef struct {
    uint16_t magic;
    uint8_t  version;
    uint8_t  payload_type;
    uint32_t sequence;
    int64_t  hw_timestamp_ns;
    uint32_t payload_length;
} fw_packet_header_t;

_Static_assert(sizeof(fw_packet_header_t) == FW_PROTO_HEADER_SIZE,
               "Header must be exactly 20 bytes");

// ─── Handshake payloads ─────────────────────────────────────────────────────

typedef struct {
    uint8_t  protocol_version;
    uint8_t  _pad[3];
    char     client_id[32];
} fw_handshake_payload_t;

typedef struct {
    uint8_t  accepted;
    uint8_t  device_protocol_version;
    uint8_t  _pad[2];
    char     serial_number[32];
    char     firmware_version[32];
    char     model_name[32];
} fw_handshake_ack_payload_t;

// ─── Heartbeat payload ──────────────────────────────────────────────────────

typedef struct {
    int64_t  sender_timestamp_ns;
    uint32_t uptime_seconds;
    uint32_t error_flags;
} fw_heartbeat_payload_t;

#pragma pack(pop)

// ─── Little-endian load helpers (portable across byte orders) ────────────────

static inline uint16_t fw_load_le16(const uint8_t* p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline uint32_t fw_load_le32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline int64_t fw_load_le64(const uint8_t* p) {
    uint64_t v = (uint64_t)p[0] | ((uint64_t)p[1] << 8) |
                 ((uint64_t)p[2] << 16) | ((uint64_t)p[3] << 24) |
                 ((uint64_t)p[4] << 32) | ((uint64_t)p[5] << 40) |
                 ((uint64_t)p[6] << 48) | ((uint64_t)p[7] << 56);
    return (int64_t)v;
}

static inline void fw_store_le16(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
}

static inline void fw_store_le32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static inline void fw_store_le64(uint8_t* p, int64_t v) {
    uint64_t u = (uint64_t)v;
    p[0] = (uint8_t)(u);
    p[1] = (uint8_t)(u >> 8);
    p[2] = (uint8_t)(u >> 16);
    p[3] = (uint8_t)(u >> 24);
    p[4] = (uint8_t)(u >> 32);
    p[5] = (uint8_t)(u >> 40);
    p[6] = (uint8_t)(u >> 48);
    p[7] = (uint8_t)(u >> 56);
}

// ─── CRC-32 (IEEE 802.3) ───────────────────────────────────────────────────

/// Compute CRC-32 over `len` bytes.
uint32_t fw_crc32(const uint8_t* data, size_t len);

/// Validate a packet: recompute CRC and compare with trailing 4 bytes.
int fw_crc32_validate(const uint8_t* packet, size_t total_len);

// ─── Packet builder ─────────────────────────────────────────────────────────

/// Build a complete packet (header + payload + CRC) into `out`.
/// Returns total bytes written, or 0 on error (buffer too small).
size_t fw_build_packet(uint8_t* out, size_t out_capacity,
                       uint8_t pkt_type, uint32_t seq,
                       int64_t hw_timestamp_ns,
                       const uint8_t* payload, uint32_t payload_len);

#ifdef __cplusplus
}
#endif
