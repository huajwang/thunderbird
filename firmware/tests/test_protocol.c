// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird Firmware — Protocol & CRC tests
// ─────────────────────────────────────────────────────────────────────────────
#include "fw/protocol.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

// ─── CRC-32 known vectors ───────────────────────────────────────────────────

static void test_crc32_empty(void) {
    uint32_t crc = fw_crc32(NULL, 0);
    // CRC-32 of empty input is 0x00000000 (init XOR final = identity)
    assert(crc == 0x00000000u);
    printf("  PASS: crc32_empty\n");
}

static void test_crc32_known_string(void) {
    // CRC-32 of "123456789" is 0xCBF43926 (standard check value)
    const uint8_t data[] = "123456789";
    uint32_t crc = fw_crc32(data, 9);
    assert(crc == 0xCBF43926u);
    printf("  PASS: crc32_known_string\n");
}

// ─── Packet builder ─────────────────────────────────────────────────────────

static void test_build_packet_minimal(void) {
    // Build a packet with zero-length payload.
    uint8_t buf[64];
    size_t n = fw_build_packet(buf, sizeof(buf),
                               FW_PKT_HEARTBEAT, 42,
                               1000000LL, NULL, 0);

    assert(n == FW_PROTO_HEADER_SIZE + FW_PROTO_CRC_SIZE);  // 24 bytes

    // Header fields
    fw_packet_header_t hdr;
    memcpy(&hdr, buf, sizeof(hdr));
    assert(hdr.magic           == FW_PROTO_MAGIC);
    assert(hdr.version         == FW_PROTO_VERSION);
    assert(hdr.payload_type    == FW_PKT_HEARTBEAT);
    assert(hdr.sequence        == 42u);
    assert(hdr.hw_timestamp_ns == 1000000LL);
    assert(hdr.payload_length  == 0u);

    // CRC must validate
    assert(fw_crc32_validate(buf, n));
    printf("  PASS: build_packet_minimal\n");
}

static void test_build_packet_with_payload(void) {
    uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF};
    uint8_t buf[64];
    size_t n = fw_build_packet(buf, sizeof(buf),
                               FW_PKT_IMU_SAMPLE, 7,
                               5000000LL, payload, 4);

    assert(n == FW_PROTO_HEADER_SIZE + 4 + FW_PROTO_CRC_SIZE);

    fw_packet_header_t hdr;
    memcpy(&hdr, buf, sizeof(hdr));
    assert(hdr.payload_type   == FW_PKT_IMU_SAMPLE);
    assert(hdr.payload_length == 4u);

    // Payload bytes intact
    assert(memcmp(buf + FW_PROTO_HEADER_SIZE, payload, 4) == 0);

    // CRC validates
    assert(fw_crc32_validate(buf, n));
    printf("  PASS: build_packet_with_payload\n");
}

static void test_build_packet_buffer_too_small(void) {
    uint8_t payload[32];
    uint8_t buf[10];  // too small for header + payload + CRC
    size_t n = fw_build_packet(buf, sizeof(buf),
                               FW_PKT_LIDAR_SCAN, 0, 0,
                               payload, 32);
    assert(n == 0);
    printf("  PASS: build_packet_buffer_too_small\n");
}

// ─── CRC validation ────────────────────────────────────────────────────────

static void test_crc32_validate_tampered(void) {
    uint8_t buf[64];
    size_t n = fw_build_packet(buf, sizeof(buf),
                               FW_PKT_HEARTBEAT, 0, 0, NULL, 0);
    assert(fw_crc32_validate(buf, n));

    // Flip a bit in the payload_type field
    buf[3] ^= 0x01;
    assert(!fw_crc32_validate(buf, n));
    printf("  PASS: crc32_validate_tampered\n");
}

static void test_crc32_validate_short(void) {
    uint8_t buf[2] = {0};
    assert(!fw_crc32_validate(buf, 2));
    printf("  PASS: crc32_validate_short\n");
}

// ─── Header size ────────────────────────────────────────────────────────────

static void test_header_size(void) {
    assert(sizeof(fw_packet_header_t) == FW_PROTO_HEADER_SIZE);
    assert(sizeof(fw_packet_header_t) == 20);
    printf("  PASS: header_size\n");
}

// ─── Sequence numbers ───────────────────────────────────────────────────────

static void test_sequence_numbers(void) {
    uint8_t buf[64];
    for (uint32_t seq = 0; seq < 5; ++seq) {
        size_t n = fw_build_packet(buf, sizeof(buf),
                                   FW_PKT_IMU_SAMPLE, seq, 0, NULL, 0);
        assert(n > 0);
        fw_packet_header_t hdr;
        memcpy(&hdr, buf, sizeof(hdr));
        assert(hdr.sequence == seq);
    }
    printf("  PASS: sequence_numbers\n");
}

// ─── Main ───────────────────────────────────────────────────────────────────

int main(void) {
    printf("=== Protocol & CRC tests ===\n");
    test_crc32_empty();
    test_crc32_known_string();
    test_build_packet_minimal();
    test_build_packet_with_payload();
    test_build_packet_buffer_too_small();
    test_crc32_validate_tampered();
    test_crc32_validate_short();
    test_header_size();
    test_sequence_numbers();
    printf("All protocol tests passed.\n");
    return 0;
}
