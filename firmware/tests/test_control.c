// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird Firmware — Control handler tests
// ─────────────────────────────────────────────────────────────────────────────
#include "fw/control.h"
#include "fw/protocol.h"
#include "fw/platform_clock.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static fw_device_identity_t test_identity(void) {
    fw_device_identity_t id;
    memset(&id, 0, sizeof(id));
    strncpy(id.serial_number,    "TEST-001",       sizeof(id.serial_number) - 1);
    strncpy(id.firmware_version, "0.0.1-test",     sizeof(id.firmware_version) - 1);
    strncpy(id.model_name,       "TestDevice",     sizeof(id.model_name) - 1);
    return id;
}

// Helper: build a control packet to send to the handler.
static size_t build_control_pkt(uint8_t* buf, size_t cap,
                                uint8_t type,
                                const uint8_t* payload, uint32_t plen) {
    return fw_build_packet(buf, cap, type, 0, 0, payload, plen);
}

// ─── Handshake ──────────────────────────────────────────────────────────────

static void test_handshake(void) {
    fw_control_ctx_t ctx;
    fw_device_identity_t id = test_identity();
    fw_control_init(&ctx, &id);

    assert(ctx.state == FW_STATE_IDLE);

    // Build a Handshake packet
    fw_handshake_payload_t hs;
    memset(&hs, 0, sizeof(hs));
    hs.protocol_version = FW_PROTO_VERSION;
    strncpy(hs.client_id, "test-client", sizeof(hs.client_id) - 1);

    uint8_t in_buf[256], out_buf[256];
    size_t in_len = build_control_pkt(in_buf, sizeof(in_buf),
                                      FW_PKT_HANDSHAKE,
                                      (const uint8_t*)&hs, sizeof(hs));
    assert(in_len > 0);

    int resp = fw_control_process(&ctx, in_buf, in_len,
                                  out_buf, sizeof(out_buf));

    // Should produce a HandshakeAck response
    assert(resp > 0);
    assert(ctx.state == FW_STATE_CONNECTED);

    // Validate the response packet
    assert(fw_crc32_validate(out_buf, (size_t)resp));

    fw_packet_header_t hdr;
    memcpy(&hdr, out_buf, sizeof(hdr));
    assert(hdr.payload_type == FW_PKT_HANDSHAKE_ACK);

    // Parse ack payload
    fw_handshake_ack_payload_t ack;
    memcpy(&ack, out_buf + FW_PROTO_HEADER_SIZE, sizeof(ack));
    assert(ack.accepted == 1);
    assert(ack.device_protocol_version == FW_PROTO_VERSION);
    assert(strcmp(ack.serial_number, "TEST-001") == 0);
    assert(strcmp(ack.firmware_version, "0.0.1-test") == 0);
    assert(strcmp(ack.model_name, "TestDevice") == 0);

    printf("  PASS: handshake\n");
}

// ─── Start / Stop stream ───────────────────────────────────────────────────

static void test_start_stop_stream(void) {
    fw_control_ctx_t ctx;
    fw_device_identity_t id = test_identity();
    fw_control_init(&ctx, &id);
    ctx.state = FW_STATE_CONNECTED;

    uint8_t in_buf[64], out_buf[64];

    // StartStream
    size_t n = build_control_pkt(in_buf, sizeof(in_buf),
                                 FW_PKT_START_STREAM, NULL, 0);
    int resp = fw_control_process(&ctx, in_buf, n, out_buf, sizeof(out_buf));
    assert(resp == 0);  // no response packet
    assert(ctx.state == FW_STATE_STREAMING);

    // StopStream
    n = build_control_pkt(in_buf, sizeof(in_buf),
                          FW_PKT_STOP_STREAM, NULL, 0);
    resp = fw_control_process(&ctx, in_buf, n, out_buf, sizeof(out_buf));
    assert(resp == 0);
    assert(ctx.state == FW_STATE_CONNECTED);

    printf("  PASS: start_stop_stream\n");
}

// ─── Start from idle (should not enter streaming) ───────────────────────────

static void test_start_from_idle(void) {
    fw_control_ctx_t ctx;
    fw_device_identity_t id = test_identity();
    fw_control_init(&ctx, &id);
    assert(ctx.state == FW_STATE_IDLE);

    uint8_t in_buf[64], out_buf[64];
    size_t n = build_control_pkt(in_buf, sizeof(in_buf),
                                 FW_PKT_START_STREAM, NULL, 0);
    fw_control_process(&ctx, in_buf, n, out_buf, sizeof(out_buf));
    // State remains IDLE — StartStream requires CONNECTED
    assert(ctx.state == FW_STATE_IDLE);

    printf("  PASS: start_from_idle\n");
}

// ─── Heartbeat ──────────────────────────────────────────────────────────────

static void test_heartbeat(void) {
    fw_control_ctx_t ctx;
    fw_device_identity_t id = test_identity();
    fw_control_init(&ctx, &id);
    ctx.state         = FW_STATE_STREAMING;
    ctx.uptime_seconds = 42;
    ctx.error_flags    = 0;

    fw_heartbeat_payload_t hb_in;
    memset(&hb_in, 0, sizeof(hb_in));
    hb_in.sender_timestamp_ns = 1234567890LL;

    uint8_t in_buf[128], out_buf[128];
    size_t n = build_control_pkt(in_buf, sizeof(in_buf),
                                 FW_PKT_HEARTBEAT,
                                 (const uint8_t*)&hb_in, sizeof(hb_in));
    int resp = fw_control_process(&ctx, in_buf, n, out_buf, sizeof(out_buf));

    assert(resp > 0);
    assert(fw_crc32_validate(out_buf, (size_t)resp));

    fw_packet_header_t hdr;
    memcpy(&hdr, out_buf, sizeof(hdr));
    assert(hdr.payload_type == FW_PKT_HEARTBEAT);

    fw_heartbeat_payload_t hb_out;
    memcpy(&hb_out, out_buf + FW_PROTO_HEADER_SIZE, sizeof(hb_out));
    assert(hb_out.uptime_seconds == 42u);
    assert(hb_out.error_flags == 0u);
    assert(hb_out.sender_timestamp_ns > 0);

    printf("  PASS: heartbeat\n");
}

// ─── Invalid packet (bad CRC) ──────────────────────────────────────────────

static void test_invalid_crc(void) {
    fw_control_ctx_t ctx;
    fw_device_identity_t id = test_identity();
    fw_control_init(&ctx, &id);

    uint8_t in_buf[64], out_buf[64];
    size_t n = build_control_pkt(in_buf, sizeof(in_buf),
                                 FW_PKT_HEARTBEAT, NULL, 0);
    // Corrupt a byte
    in_buf[5] ^= 0xFF;

    int resp = fw_control_process(&ctx, in_buf, n, out_buf, sizeof(out_buf));
    assert(resp == -1);  // rejected

    printf("  PASS: invalid_crc\n");
}

// ─── Packet too short ───────────────────────────────────────────────────────

static void test_short_packet(void) {
    fw_control_ctx_t ctx;
    fw_device_identity_t id = test_identity();
    fw_control_init(&ctx, &id);

    uint8_t in_buf[4] = {0xEF, 0xBE, 0x01, 0x10};
    uint8_t out_buf[64];
    int resp = fw_control_process(&ctx, in_buf, sizeof(in_buf),
                                  out_buf, sizeof(out_buf));
    assert(resp == -1);

    printf("  PASS: short_packet\n");
}

// ─── Unknown packet type ───────────────────────────────────────────────────

static void test_unknown_type(void) {
    fw_control_ctx_t ctx;
    fw_device_identity_t id = test_identity();
    fw_control_init(&ctx, &id);
    ctx.state = FW_STATE_CONNECTED;

    uint8_t in_buf[64], out_buf[64];
    size_t n = build_control_pkt(in_buf, sizeof(in_buf),
                                 0xAA,  // unknown type
                                 NULL, 0);
    int resp = fw_control_process(&ctx, in_buf, n, out_buf, sizeof(out_buf));
    assert(resp == 0);  // silently ignored

    printf("  PASS: unknown_type\n");
}

// ─── Main ───────────────────────────────────────────────────────────────────

int main(void) {
    printf("=== Control handler tests ===\n");
    test_handshake();
    test_start_stop_stream();
    test_start_from_idle();
    test_heartbeat();
    test_invalid_crc();
    test_short_packet();
    test_unknown_type();
    printf("All control tests passed.\n");
    return 0;
}
