// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird Firmware — Control protocol handler implementation
// ─────────────────────────────────────────────────────────────────────────────
#include "fw/control.h"
#include "fw/platform_clock.h"

#include <string.h>

void fw_control_init(fw_control_ctx_t* ctx,
                     const fw_device_identity_t* id) {
    memset(ctx, 0, sizeof(*ctx));
    ctx->state = FW_STATE_IDLE;
    ctx->identity = *id;
}

int fw_control_process(fw_control_ctx_t* ctx,
                       const uint8_t* in_buf, size_t in_len,
                       uint8_t* out_buf, size_t out_capacity) {
    // Minimum viable packet: header + CRC
    if (in_len < FW_PROTO_HEADER_SIZE + FW_PROTO_CRC_SIZE)
        return -1;

    // Validate CRC
    if (!fw_crc32_validate(in_buf, in_len))
        return -1;

    // Parse header using little-endian loads for portability on big-endian targets
    uint16_t magic          = fw_load_le16(in_buf);
    uint8_t  version        = in_buf[2];
    uint8_t  payload_type   = in_buf[3];
    uint32_t payload_length = fw_load_le32(in_buf + 16);

    if (magic != FW_PROTO_MAGIC)
        return -1;

    if (version != FW_PROTO_VERSION)
        return -1;

    // Validate payload_length matches actual data received
    size_t expected_len = FW_PROTO_HEADER_SIZE + payload_length + FW_PROTO_CRC_SIZE;
    if (in_len != expected_len)
        return -1;

    if (payload_length > FW_PROTO_MAX_PAYLOAD)
        return -1;

    switch (payload_type) {

    // ── Handshake ───────────────────────────────────────────────────────
    case FW_PKT_HANDSHAKE: {
        // Accept any valid handshake regardless of current state.
        ctx->state = FW_STATE_CONNECTED;

        fw_handshake_ack_payload_t ack;
        memset(&ack, 0, sizeof(ack));
        ack.accepted = 1;
        ack.device_protocol_version = FW_PROTO_VERSION;
        strncpy(ack.serial_number,    ctx->identity.serial_number,
                sizeof(ack.serial_number) - 1);
        strncpy(ack.firmware_version, ctx->identity.firmware_version,
                sizeof(ack.firmware_version) - 1);
        strncpy(ack.model_name,       ctx->identity.model_name,
                sizeof(ack.model_name) - 1);

        size_t n = fw_build_packet(out_buf, out_capacity,
                                   FW_PKT_HANDSHAKE_ACK, 0,
                                   (int64_t)fw_clock_now_ns(),
                                   (const uint8_t*)&ack, sizeof(ack));
        return (int)n;
    }

    // ── StartStream ─────────────────────────────────────────────────────
    case FW_PKT_START_STREAM: {
        if (ctx->state == FW_STATE_CONNECTED ||
            ctx->state == FW_STATE_STREAMING) {
            ctx->state = FW_STATE_STREAMING;
        }
        return 0;  // no response packet
    }

    // ── StopStream ──────────────────────────────────────────────────────
    case FW_PKT_STOP_STREAM: {
        if (ctx->state == FW_STATE_STREAMING)
            ctx->state = FW_STATE_CONNECTED;
        return 0;
    }

    // ── Heartbeat ───────────────────────────────────────────────────────
    case FW_PKT_HEARTBEAT: {
        fw_heartbeat_payload_t reply;
        memset(&reply, 0, sizeof(reply));
        reply.sender_timestamp_ns = (int64_t)fw_clock_now_ns();
        reply.uptime_seconds      = ctx->uptime_seconds;
        reply.error_flags         = ctx->error_flags;

        size_t n = fw_build_packet(out_buf, out_capacity,
                                   FW_PKT_HEARTBEAT, 0,
                                   reply.sender_timestamp_ns,
                                   (const uint8_t*)&reply, sizeof(reply));
        return (int)n;
    }

    default:
        return 0;  // unknown packet type — ignore
    }
}
