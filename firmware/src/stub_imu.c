// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird Firmware — Stub IMU driver
// ─────────────────────────────────────────────────────────────────────────────
//
// Generates synthetic IMU samples for integration testing with the
// host-side SDK.  Replace with a real chip driver when hardware is selected.
// ─────────────────────────────────────────────────────────────────────────────
#include "fw/sensor_hal.h"
#include "fw/protocol.h"
#include "fw/platform_clock.h"

#include <math.h>
#include <string.h>

// ─── Private state ──────────────────────────────────────────────────────────

#define STUB_IMU_INTERVAL_NS 5000000ULL  // 5 ms → 200 Hz

typedef struct {
    uint64_t last_emit_ns;
    uint32_t sample_count;
} stub_imu_state_t;

static stub_imu_state_t s_imu_state;

// ─── Driver operations ─────────────────────────────────────────────────────

static int stub_imu_init(sensor_driver_t* self) {
    stub_imu_state_t* st = (stub_imu_state_t*)self->priv;
    st->last_emit_ns = 0;
    st->sample_count = 0;
    return 0;
}

static int stub_imu_read(sensor_driver_t* self,
                         uint8_t* buf, size_t max_len,
                         uint64_t* timestamp_ns) {
    stub_imu_state_t* st = (stub_imu_state_t*)self->priv;
    uint64_t now = fw_clock_now_ns();

    if (st->last_emit_ns != 0 &&
        (now - st->last_emit_ns) < STUB_IMU_INTERVAL_NS) {
        return 0;  // not time yet
    }
    st->last_emit_ns = now;
    *timestamp_ns = now;

    // ImuWirePayload: 7 floats × 4 bytes = 28 bytes
    if (max_len < 28) return -1;

    float t = (float)st->sample_count * 0.005f;  // time in seconds
    float accel_x = 0.01f * sinf(t);
    float accel_y = 0.01f * cosf(t);
    float accel_z = 9.81f;
    float gyro_x  = 0.001f * sinf(t * 2.0f);
    float gyro_y  = 0.001f * cosf(t * 2.0f);
    float gyro_z  = 0.0f;
    float temp    = 25.0f + 0.1f * sinf(t * 0.1f);

    memcpy(buf + 0,  &accel_x, 4);
    memcpy(buf + 4,  &accel_y, 4);
    memcpy(buf + 8,  &accel_z, 4);
    memcpy(buf + 12, &gyro_x,  4);
    memcpy(buf + 16, &gyro_y,  4);
    memcpy(buf + 20, &gyro_z,  4);
    memcpy(buf + 24, &temp,    4);

    st->sample_count++;
    return 28;
}

static void stub_imu_shutdown(sensor_driver_t* self) {
    (void)self;
}

// ─── Public driver instance ─────────────────────────────────────────────────

sensor_driver_t g_stub_imu_driver = {
    .name     = "stub-imu",
    .type     = SENSOR_IMU,
    .init     = stub_imu_init,
    .read     = stub_imu_read,
    .shutdown = stub_imu_shutdown,
    .priv     = &s_imu_state,
};
