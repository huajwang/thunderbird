// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird Firmware — Stub LiDAR driver
// ─────────────────────────────────────────────────────────────────────────────
//
// Generates synthetic LiDAR scan data for integration testing with the
// host-side SDK.  Replace with a real chip driver when hardware is selected.
// ─────────────────────────────────────────────────────────────────────────────
#include "fw/sensor_hal.h"
#include "fw/protocol.h"
#include "fw/platform_clock.h"

#include <math.h>
#include <string.h>

// ─── Private state ──────────────────────────────────────────────────────────

#define STUB_LIDAR_NUM_POINTS  64
#define STUB_LIDAR_INTERVAL_NS 100000000ULL  // 100 ms → 10 Hz

typedef struct {
    uint64_t last_emit_ns;
    float    azimuth;
} stub_lidar_state_t;

static stub_lidar_state_t s_lidar_state;

// ─── Driver operations ─────────────────────────────────────────────────────

static int stub_lidar_init(sensor_driver_t* self) {
    stub_lidar_state_t* st = (stub_lidar_state_t*)self->priv;
    st->last_emit_ns = 0;
    st->azimuth      = 0.0f;
    return 0;
}

static int stub_lidar_read(sensor_driver_t* self,
                           uint8_t* buf, size_t max_len,
                           uint64_t* timestamp_ns) {
    stub_lidar_state_t* st = (stub_lidar_state_t*)self->priv;
    uint64_t now = fw_clock_now_ns();

    if (st->last_emit_ns != 0 &&
        (now - st->last_emit_ns) < STUB_LIDAR_INTERVAL_NS) {
        return 0;  // not time yet
    }
    st->last_emit_ns = now;
    *timestamp_ns = now;

    // Build a LiDAR sub-header + points payload matching the wire format.
    // Sub-header: num_points(4) + azimuth_start(4) + azimuth_end(4) = 12 bytes
    // Each point: x(4) + y(4) + z(4) + intensity(1) + ring(1) + pad(2) = 16 bytes
    size_t sub_hdr_size = 12;
    size_t points_size  = STUB_LIDAR_NUM_POINTS * 16;
    size_t total        = sub_hdr_size + points_size;

    if (max_len < total) return -1;

    memset(buf, 0, total);

    // Sub-header
    uint32_t np = STUB_LIDAR_NUM_POINTS;
    float az_start_deg = st->azimuth;                  // degrees
    float az_end_deg   = st->azimuth + 360.0f;          // full 360° sweep
    memcpy(buf + 0, &np,           4);
    memcpy(buf + 4, &az_start_deg, 4);
    memcpy(buf + 8, &az_end_deg,   4);

    // Synthetic points: ring pattern at fixed radius
    uint8_t* pts = buf + sub_hdr_size;
    float deg2rad = 3.14159265f / 180.0f;
    for (uint32_t i = 0; i < STUB_LIDAR_NUM_POINTS; ++i) {
        float angle_deg = az_start_deg + (float)i * (360.0f / STUB_LIDAR_NUM_POINTS);
        float angle = angle_deg * deg2rad;
        float r = 5.0f;
        float x = r * cosf(angle);
        float y = r * sinf(angle);
        float z = 0.1f * (float)i;
        uint8_t intensity = (uint8_t)(128 + 64 * sinf(angle));
        uint8_t ring      = (uint8_t)(i % 16);

        memcpy(pts + i * 16 + 0,  &x, 4);
        memcpy(pts + i * 16 + 4,  &y, 4);
        memcpy(pts + i * 16 + 8,  &z, 4);
        pts[i * 16 + 12] = intensity;
        pts[i * 16 + 13] = ring;
    }

    st->azimuth += 360.0f;
    if (st->azimuth >= 36000.0f) st->azimuth = 0.0f;

    return (int)total;
}

static void stub_lidar_shutdown(sensor_driver_t* self) {
    (void)self;
}

// ─── Public driver instance ─────────────────────────────────────────────────

sensor_driver_t g_stub_lidar_driver = {
    .name     = "stub-lidar",
    .type     = SENSOR_LIDAR,
    .init     = stub_lidar_init,
    .read     = stub_lidar_read,
    .shutdown = stub_lidar_shutdown,
    .priv     = &s_lidar_state,
};
