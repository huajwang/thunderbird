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

    uint32_t stored = 0;
    memcpy(&stored, packet + total_len - FW_PROTO_CRC_SIZE,
           FW_PROTO_CRC_SIZE);
    uint32_t computed = fw_crc32(packet, total_len - FW_PROTO_CRC_SIZE);
    return stored == computed;
}

// ─── Packet builder ─────────────────────────────────────────────────────────

size_t fw_build_packet(uint8_t* out, size_t out_capacity,
                       uint8_t pkt_type, uint32_t seq,
                       int64_t hw_timestamp_ns,
                       const uint8_t* payload, uint32_t payload_len) {
    size_t total = FW_PROTO_HEADER_SIZE + payload_len + FW_PROTO_CRC_SIZE;
    if (total > out_capacity) return 0;

    fw_packet_header_t hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.magic           = FW_PROTO_MAGIC;
    hdr.version         = FW_PROTO_VERSION;
    hdr.payload_type    = pkt_type;
    hdr.sequence        = seq;
    hdr.hw_timestamp_ns = hw_timestamp_ns;
    hdr.payload_length  = payload_len;

    memcpy(out, &hdr, FW_PROTO_HEADER_SIZE);
    if (payload_len > 0 && payload != NULL)
        memcpy(out + FW_PROTO_HEADER_SIZE, payload, payload_len);

    uint32_t crc = fw_crc32(out, FW_PROTO_HEADER_SIZE + payload_len);
    memcpy(out + FW_PROTO_HEADER_SIZE + payload_len, &crc,
           FW_PROTO_CRC_SIZE);

    return total;
}
