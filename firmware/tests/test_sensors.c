// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird Firmware — Stub sensor driver tests
// ─────────────────────────────────────────────────────────────────────────────
#include "fw/sensor_hal.h"
#include "fw/protocol.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
static void sleep_ms(uint32_t ms) { Sleep(ms); }
#else
#include <unistd.h>
static void sleep_ms(uint32_t ms) { usleep(ms * 1000); }
#endif

// Extern stub drivers
extern sensor_driver_t g_stub_lidar_driver;
extern sensor_driver_t g_stub_imu_driver;
extern sensor_driver_t g_stub_camera_driver;

// ─── LiDAR driver ───────────────────────────────────────────────────────────

static void test_lidar_init(void) {
    int rc = g_stub_lidar_driver.init(&g_stub_lidar_driver);
    assert(rc == 0);
    assert(g_stub_lidar_driver.type == SENSOR_LIDAR);
    assert(g_stub_lidar_driver.name != NULL);
    printf("  PASS: lidar_init\n");
}

static void test_lidar_read(void) {
    g_stub_lidar_driver.init(&g_stub_lidar_driver);

    uint8_t buf[4096];
    uint64_t ts = 0;

    // First read should produce data (since last_emit_ns starts at 0).
    int n = g_stub_lidar_driver.read(&g_stub_lidar_driver,
                                     buf, sizeof(buf), &ts);
    assert(n > 0);
    assert(ts > 0);

    // Payload should contain the sub-header (12 bytes) + points.
    // Sub-header: num_points(4) + azimuth_start(4) + azimuth_end(4)
    assert(n >= 12);
    uint32_t num_points;
    memcpy(&num_points, buf, 4);
    assert(num_points == 64);
    assert(n == 12 + (int)(num_points * 16));  // sub-header + 64 × 16

    // Immediate re-read should return 0 (rate-limited).
    int n2 = g_stub_lidar_driver.read(&g_stub_lidar_driver,
                                      buf, sizeof(buf), &ts);
    assert(n2 == 0);

    printf("  PASS: lidar_read\n");
}

static void test_lidar_buffer_too_small(void) {
    g_stub_lidar_driver.init(&g_stub_lidar_driver);
    uint8_t buf[8];  // way too small
    uint64_t ts = 0;
    int n = g_stub_lidar_driver.read(&g_stub_lidar_driver,
                                     buf, sizeof(buf), &ts);
    assert(n == -1);  // error: buffer too small
    printf("  PASS: lidar_buffer_too_small\n");
}

// ─── IMU driver ─────────────────────────────────────────────────────────────

static void test_imu_init(void) {
    int rc = g_stub_imu_driver.init(&g_stub_imu_driver);
    assert(rc == 0);
    assert(g_stub_imu_driver.type == SENSOR_IMU);
    printf("  PASS: imu_init\n");
}

static void test_imu_read(void) {
    g_stub_imu_driver.init(&g_stub_imu_driver);

    uint8_t buf[64];
    uint64_t ts = 0;

    int n = g_stub_imu_driver.read(&g_stub_imu_driver,
                                   buf, sizeof(buf), &ts);
    assert(n == 28);  // ImuWirePayload: 7 floats × 4 bytes
    assert(ts > 0);

    // Verify accel_z ≈ 9.81 (gravity).
    float accel_z;
    memcpy(&accel_z, buf + 8, 4);
    assert(accel_z > 9.5f && accel_z < 10.1f);

    // Immediate re-read should return 0 (5 ms interval).
    int n2 = g_stub_imu_driver.read(&g_stub_imu_driver,
                                    buf, sizeof(buf), &ts);
    assert(n2 == 0);

    printf("  PASS: imu_read\n");
}

static void test_imu_rate_limited(void) {
    g_stub_imu_driver.init(&g_stub_imu_driver);

    uint8_t buf[64];
    uint64_t ts = 0;

    // First read
    g_stub_imu_driver.read(&g_stub_imu_driver, buf, sizeof(buf), &ts);

    // Wait 10 ms (> 5 ms interval) and read again
    sleep_ms(10);
    int n = g_stub_imu_driver.read(&g_stub_imu_driver,
                                   buf, sizeof(buf), &ts);
    assert(n == 28);

    printf("  PASS: imu_rate_limited\n");
}

// ─── Camera driver ──────────────────────────────────────────────────────────

static void test_camera_init(void) {
    int rc = g_stub_camera_driver.init(&g_stub_camera_driver);
    assert(rc == 0);
    assert(g_stub_camera_driver.type == SENSOR_CAMERA);
    printf("  PASS: camera_init\n");
}

static void test_camera_read(void) {
    g_stub_camera_driver.init(&g_stub_camera_driver);

    // 160×120×3 = 57600 + 16 sub-header = 57616
    uint8_t* buf = (uint8_t*)malloc(65536);
    assert(buf != NULL);
    uint64_t ts = 0;

    int n = g_stub_camera_driver.read(&g_stub_camera_driver,
                                      buf, 65536, &ts);
    assert(n == 57616);
    assert(ts > 0);

    // Validate sub-header
    uint32_t width, height, data_len;
    uint8_t pixel_fmt;
    memcpy(&width,    buf + 0,  4);
    memcpy(&height,   buf + 4,  4);
    pixel_fmt = buf[8];
    memcpy(&data_len, buf + 12, 4);
    assert(width  == 160);
    assert(height == 120);
    assert(pixel_fmt == 1);  // RGB8 = 1
    assert(data_len == 160u * 120u * 3u);

    free(buf);
    printf("  PASS: camera_read\n");
}

static void test_camera_buffer_too_small(void) {
    g_stub_camera_driver.init(&g_stub_camera_driver);
    uint8_t buf[32];
    uint64_t ts = 0;
    int n = g_stub_camera_driver.read(&g_stub_camera_driver,
                                      buf, sizeof(buf), &ts);
    assert(n == -1);
    printf("  PASS: camera_buffer_too_small\n");
}

// ─── HAL interface contract ─────────────────────────────────────────────────

static void test_all_drivers_have_vtable(void) {
    sensor_driver_t* drivers[] = {
        &g_stub_lidar_driver,
        &g_stub_imu_driver,
        &g_stub_camera_driver,
    };
    for (size_t i = 0; i < 3; ++i) {
        assert(drivers[i]->name     != NULL);
        assert(drivers[i]->init     != NULL);
        assert(drivers[i]->read     != NULL);
        assert(drivers[i]->shutdown != NULL);
    }
    printf("  PASS: all_drivers_have_vtable\n");
}

static void test_shutdown_is_safe(void) {
    g_stub_lidar_driver.shutdown(&g_stub_lidar_driver);
    g_stub_imu_driver.shutdown(&g_stub_imu_driver);
    g_stub_camera_driver.shutdown(&g_stub_camera_driver);
    printf("  PASS: shutdown_is_safe\n");
}

// ─── Main ───────────────────────────────────────────────────────────────────

int main(void) {
    printf("=== Stub sensor driver tests ===\n");
    test_all_drivers_have_vtable();
    test_lidar_init();
    test_lidar_read();
    test_lidar_buffer_too_small();
    test_imu_init();
    test_imu_read();
    test_imu_rate_limited();
    test_camera_init();
    test_camera_read();
    test_camera_buffer_too_small();
    test_shutdown_is_safe();
    printf("All sensor driver tests passed.\n");
    return 0;
}
