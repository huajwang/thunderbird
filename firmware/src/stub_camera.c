// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird Firmware — Stub camera driver
// ─────────────────────────────────────────────────────────────────────────────
//
// Generates synthetic camera frames for integration testing with the
// host-side SDK.  Replace with a real chip driver when hardware is selected.
// ─────────────────────────────────────────────────────────────────────────────
#include "fw/sensor_hal.h"
#include "fw/protocol.h"
#include "fw/platform_clock.h"

#include <string.h>

// ─── Private state ──────────────────────────────────────────────────────────

#define STUB_CAM_WIDTH       160
#define STUB_CAM_HEIGHT      120
#define STUB_CAM_BPP         3        // RGB8
#define STUB_CAM_INTERVAL_NS 33333333ULL  // ~30 fps

typedef struct {
    uint64_t last_emit_ns;
    uint32_t frame_count;
} stub_camera_state_t;

static stub_camera_state_t s_camera_state;

// ─── Driver operations ─────────────────────────────────────────────────────

static int stub_camera_init(sensor_driver_t* self) {
    stub_camera_state_t* st = (stub_camera_state_t*)self->priv;
    st->last_emit_ns = 0;
    st->frame_count  = 0;
    return 0;
}

static int stub_camera_read(sensor_driver_t* self,
                            uint8_t* buf, size_t max_len,
                            uint64_t* timestamp_ns) {
    stub_camera_state_t* st = (stub_camera_state_t*)self->priv;
    uint64_t now = fw_clock_now_ns();

    if (st->last_emit_ns != 0 &&
        (now - st->last_emit_ns) < STUB_CAM_INTERVAL_NS) {
        return 0;  // not time yet
    }
    st->last_emit_ns = now;
    *timestamp_ns = now;

    // CameraSubHeader: width(4) + height(4) + pixel_format(1) + pad(3)
    //                  + data_length(4) = 16 bytes
    uint32_t pixel_data_len = STUB_CAM_WIDTH * STUB_CAM_HEIGHT * STUB_CAM_BPP;
    size_t   total          = 16 + pixel_data_len;

    if (max_len < total) return -1;

    memset(buf, 0, 16);

    uint32_t w   = STUB_CAM_WIDTH;
    uint32_t h   = STUB_CAM_HEIGHT;
    uint8_t  fmt = 0;  // RGB8

    memcpy(buf + 0,  &w,   4);
    memcpy(buf + 4,  &h,   4);
    buf[8] = fmt;
    memcpy(buf + 12, &pixel_data_len, 4);

    // Synthetic gradient pattern that shifts each frame.
    uint8_t* pixels = buf + 16;
    uint8_t  shift  = (uint8_t)(st->frame_count & 0xFF);
    for (uint32_t row = 0; row < STUB_CAM_HEIGHT; ++row) {
        for (uint32_t col = 0; col < STUB_CAM_WIDTH; ++col) {
            size_t idx = (row * STUB_CAM_WIDTH + col) * STUB_CAM_BPP;
            pixels[idx + 0] = (uint8_t)(col + shift);        // R
            pixels[idx + 1] = (uint8_t)(row + shift);        // G
            pixels[idx + 2] = (uint8_t)(col + row + shift);  // B
        }
    }

    st->frame_count++;
    return (int)total;
}

static void stub_camera_shutdown(sensor_driver_t* self) {
    (void)self;
}

// ─── Public driver instance ─────────────────────────────────────────────────

sensor_driver_t g_stub_camera_driver = {
    .name     = "stub-camera",
    .type     = SENSOR_CAMERA,
    .init     = stub_camera_init,
    .read     = stub_camera_read,
    .shutdown = stub_camera_shutdown,
    .priv     = &s_camera_state,
};
