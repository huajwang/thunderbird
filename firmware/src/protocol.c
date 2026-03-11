// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird Firmware — Protocol framing implementation
// ─────────────────────────────────────────────────────────────────────────────
#include "fw/protocol.h"

#include <string.h>

// ─── CRC-32 table (IEEE 802.3, same polynomial as SDK) ─────────────────────

static uint32_t s_crc32_table[256];
static int      s_crc32_init = 0;

static void crc32_build_table(void) {
    for (uint32_t i = 0; i < 256; ++i) {
        uint32_t c = i;
        for (int j = 0; j < 8; ++j)
            c = (c & 1u) ? (0xEDB88320u ^ (c >> 1u)) : (c >> 1u);
        s_crc32_table[i] = c;
    }
    s_crc32_init = 1;
}

uint32_t fw_crc32(const uint8_t* data, size_t len) {
    if (!s_crc32_init) crc32_build_table();

    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i)
        crc = s_crc32_table[(crc ^ data[i]) & 0xFFu] ^ (crc >> 8u);
    return crc ^ 0xFFFFFFFFu;
}

int fw_crc32_validate(const uint8_t* packet, size_t total_len) {
    if (total_len < FW_PROTO_CRC_SIZE) return 0;

    uint32_t stored = fw_load_le32(packet + total_len - FW_PROTO_CRC_SIZE);
    uint32_t computed = fw_crc32(packet, total_len - FW_PROTO_CRC_SIZE);
    return stored == computed;
}

// ─── Packet builder ─────────────────────────────────────────────────────────

size_t fw_build_packet(uint8_t* out, size_t out_capacity,
                       uint8_t pkt_type, uint32_t seq,
                       int64_t hw_timestamp_ns,
                       const uint8_t* payload, uint32_t payload_len) {
    if (payload_len > 0 && payload == NULL) return 0;
    if (payload_len > FW_PROTO_MAX_PAYLOAD) return 0;
    size_t total = FW_PROTO_HEADER_SIZE + payload_len + FW_PROTO_CRC_SIZE;
    if (total > out_capacity) return 0;

    // Write header fields in little-endian for portability
    memset(out, 0, FW_PROTO_HEADER_SIZE);
    fw_store_le16(out, FW_PROTO_MAGIC);
    out[2] = FW_PROTO_VERSION;
    out[3] = pkt_type;
    fw_store_le32(out + 4, seq);
    fw_store_le64(out + 8, hw_timestamp_ns);
    fw_store_le32(out + 16, payload_len);

    if (payload_len > 0 && payload != NULL)
        memcpy(out + FW_PROTO_HEADER_SIZE, payload, payload_len);

    uint32_t crc = fw_crc32(out, FW_PROTO_HEADER_SIZE + payload_len);
    fw_store_le32(out + FW_PROTO_HEADER_SIZE + payload_len, crc);

    return total;
}
