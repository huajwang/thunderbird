// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird Firmware — Sensor multiplexer implementation
// ─────────────────────────────────────────────────────────────────────────────
//
// Main run-loop:
//   1. Accept host connection and wait for handshake.
//   2. Enter streaming: poll sensors, wrap in packets, send.
//   3. Periodically check for incoming control packets.
//   4. Exit on disconnect or unrecoverable error.
// ─────────────────────────────────────────────────────────────────────────────
#include "fw/mux.h"
#include "fw/protocol.h"
#include "fw/platform_clock.h"

#include <string.h>

#ifdef _WIN32
  #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
  #endif
  #include <windows.h>
  #define mux_sleep_ms(ms)  Sleep(ms)
#else
  #include <unistd.h>
  #define mux_sleep_ms(ms)  usleep((ms) * 1000u)
#endif

// Map sensor_type_t → PacketType byte
static uint8_t sensor_pkt_type(sensor_type_t t) {
    switch (t) {
        case SENSOR_LIDAR:  return FW_PKT_LIDAR_SCAN;
        case SENSOR_IMU:    return FW_PKT_IMU_SAMPLE;
        case SENSOR_CAMERA: return FW_PKT_CAMERA_FRAME;
        default:            return FW_PKT_ERROR;
    }
}

// Maximum scratch buffer for a single outbound packet.
// Camera frames are the biggest: 16-byte sub-header + pixel data.
// 160×120×3 = 57600 + 16 = 57616 payload + 24 framing ≈ 58 KB.
#define MUX_SENSOR_BUF_SIZE   (64u * 1024u)
#define MUX_PKT_BUF_SIZE      (MUX_SENSOR_BUF_SIZE + FW_PROTO_HEADER_SIZE + FW_PROTO_CRC_SIZE)
#define MUX_CTRL_BUF_SIZE     512u

int fw_mux_run(sensor_driver_t* sensors[], size_t n_sensors,
               fw_transport_t* transport,
               const fw_device_identity_t* identity,
               const fw_mux_config_t* config) {
    // ── Initialise control context ──────────────────────────────────────
    fw_control_ctx_t ctrl;
    fw_control_init(&ctrl, identity);

    // ── Per-sensor sequence counters ────────────────────────────────────
    uint32_t seq[SENSOR_COUNT];
    memset(seq, 0, sizeof(seq));

    // ── Scratch buffers (static to avoid large stack usage on MCUs) ─────
    static uint8_t sensor_buf[MUX_SENSOR_BUF_SIZE];
    static uint8_t pkt_buf[MUX_PKT_BUF_SIZE];
    static uint8_t ctrl_in[MUX_CTRL_BUF_SIZE];
    static uint8_t ctrl_out[MUX_CTRL_BUF_SIZE];

    // ── Initialise sensor drivers ───────────────────────────────────────
    for (size_t i = 0; i < n_sensors; ++i) {
        int rc = sensors[i]->init(sensors[i]);
        if (rc != 0) return rc;
    }

    // ── Wait for host connection ────────────────────────────────────────
    if (fw_transport_accept(transport) != 0)
        return -1;

    // ── Main loop ───────────────────────────────────────────────────────
    uint64_t last_ctrl_poll_ns = 0;
    uint64_t ctrl_poll_interval_ns =
        (uint64_t)config->control_poll_interval_ms * 1000000ULL;

    while (fw_transport_is_connected(transport)) {
        uint64_t now = fw_clock_now_ns();

        // ── Poll for incoming control packets ───────────────────────────
        if ((now - last_ctrl_poll_ns) >= ctrl_poll_interval_ns) {
            last_ctrl_poll_ns = now;

            int n = fw_transport_recv(transport, ctrl_in,
                                      sizeof(ctrl_in), 0);
            if (n > 0) {
                int resp = fw_control_process(&ctrl,
                                              ctrl_in, (size_t)n,
                                              ctrl_out, sizeof(ctrl_out));
                if (resp > 0) {
                    fw_transport_send(transport, ctrl_out, (size_t)resp);
                }
            } else if (n < 0) {
                break;  // host disconnected
            }
        }

        // ── Stream sensor data when in STREAMING state ──────────────────
        if (ctrl.state != FW_STATE_STREAMING) {
            // Avoid busy-spinning while waiting for StartStream.
            // Block on recv with a short timeout instead of spinning.
            int n = fw_transport_recv(transport, ctrl_in,
                                      sizeof(ctrl_in), 50);
            if (n > 0) {
                int resp = fw_control_process(&ctrl,
                                              ctrl_in, (size_t)n,
                                              ctrl_out, sizeof(ctrl_out));
                if (resp > 0) {
                    fw_transport_send(transport, ctrl_out, (size_t)resp);
                }
            } else if (n < 0) {
                break;
            }
            continue;
        }

        int any_data = 0;
        for (size_t i = 0; i < n_sensors; ++i) {
            uint64_t ts = 0;
            int payload_len = sensors[i]->read(sensors[i],
                                               sensor_buf,
                                               sizeof(sensor_buf),
                                               &ts);
            if (payload_len <= 0) continue;

            any_data = 1;
            uint8_t ptype = sensor_pkt_type(sensors[i]->type);
            size_t pkt_len = fw_build_packet(pkt_buf, sizeof(pkt_buf),
                                             ptype,
                                             seq[sensors[i]->type]++,
                                             (int64_t)ts,
                                             sensor_buf,
                                             (uint32_t)payload_len);
            if (pkt_len == 0) continue;

            // Sensor data goes over the data channel (UDP when available)
            int sent = fw_transport_send_data(transport, pkt_buf, pkt_len);
            if (sent < 0) goto done;  // host disconnected
        }

        // Avoid busy-spinning when no sensor produced data this iteration
        if (!any_data)
            mux_sleep_ms(1);
    }

done:
    // ── Shutdown sensors ────────────────────────────────────────────────
    for (size_t i = 0; i < n_sensors; ++i)
        sensors[i]->shutdown(sensors[i]);

    fw_transport_close_client(transport);
    return 0;
}
