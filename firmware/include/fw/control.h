// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird Firmware — Control protocol handler
// ─────────────────────────────────────────────────────────────────────────────
//
// Processes control packets from the host (Handshake, StartStream,
// StopStream, Heartbeat) and produces the appropriate response packets.
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "fw/protocol.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ─── Device identity (set once at boot) ─────────────────────────────────────

typedef struct {
    char serial_number[32];
    char firmware_version[32];
    char model_name[32];
} fw_device_identity_t;

// ─── Connection state ───────────────────────────────────────────────────────

typedef enum {
    FW_STATE_IDLE,            // waiting for handshake
    FW_STATE_CONNECTED,       // handshake done, not streaming
    FW_STATE_STREAMING,       // actively sending sensor data
} fw_conn_state_t;

typedef struct {
    fw_conn_state_t      state;
    fw_device_identity_t identity;
    uint32_t             uptime_seconds;
    uint32_t             error_flags;
} fw_control_ctx_t;

/// Initialise the control context with device identity.
void fw_control_init(fw_control_ctx_t* ctx, const fw_device_identity_t* id);

/// Process one incoming control packet.
/// Reads `in_buf` (a full packet including header + CRC), performs the
/// appropriate state transition, and writes a response packet into `out_buf`.
/// Returns bytes written to out_buf (0 if no response needed), or negative
/// on error.
int fw_control_process(fw_control_ctx_t* ctx,
                       const uint8_t* in_buf, size_t in_len,
                       uint8_t* out_buf, size_t out_capacity);

#ifdef __cplusplus
}
#endif
