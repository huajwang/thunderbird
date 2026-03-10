// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird Firmware — Sensor multiplexer
// ─────────────────────────────────────────────────────────────────────────────
//
// The multiplexer is the firmware's main run-loop.  It:
//   1. Accepts a host connection and completes the handshake.
//   2. Polls all registered sensors via the HAL interface.
//   3. Wraps each reading in a Thunderbird protocol packet.
//   4. Sends packets over the single transport link.
//   5. Handles incoming control packets (heartbeat, stop, etc.).
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "fw/sensor_hal.h"
#include "fw/transport.h"
#include "fw/control.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/// Configuration for the multiplexer run-loop.
typedef struct {
    /// How often to poll for incoming control packets (ms).
    uint32_t control_poll_interval_ms;
} fw_mux_config_t;

/// Default configuration.
static inline fw_mux_config_t fw_mux_default_config(void) {
    fw_mux_config_t cfg = {0};
    cfg.control_poll_interval_ms = 10;
    return cfg;
}

/// Run the firmware main loop.
/// Blocks until the host disconnects or an unrecoverable error occurs.
///
/// @param sensors     Array of sensor drivers (one per type).
/// @param n_sensors   Number of entries in `sensors`.
/// @param transport   Host transport (already listening, not yet accepted).
/// @param identity    Device identity for handshake response.
/// @param config      Multiplexer configuration.
/// @return            0 on clean shutdown, negative on error.
int fw_mux_run(sensor_driver_t* sensors[], size_t n_sensors,
               fw_transport_t* transport,
               const fw_device_identity_t* identity,
               const fw_mux_config_t* config);

#ifdef __cplusplus
}
#endif
