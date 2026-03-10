// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird Firmware — Main entry point
// ─────────────────────────────────────────────────────────────────────────────
//
// Boots the firmware with stub sensor drivers and a TCP transport for
// desktop integration testing.  On real hardware, replace the transport
// and sensor drivers with production implementations.
//
// Usage:
//   ./thunderbird_firmware [port]
//
// Default port is 5555.  The SDK can connect with uri "eth://127.0.0.1:5555".
// ─────────────────────────────────────────────────────────────────────────────
#include "fw/mux.h"
#include "fw/sensor_hal.h"
#include "fw/transport.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Extern stub driver instances defined in stub_*.c
extern sensor_driver_t g_stub_lidar_driver;
extern sensor_driver_t g_stub_imu_driver;
extern sensor_driver_t g_stub_camera_driver;

int main(int argc, const char* argv[]) {
    uint16_t port = 5555;
    if (argc > 1) {
        int p = atoi(argv[1]);
        if (p > 0 && p < 65536) port = (uint16_t)p;
    }

    // ── Device identity ─────────────────────────────────────────────────
    fw_device_identity_t id;
    memset(&id, 0, sizeof(id));
    strncpy(id.serial_number,    "TB-FW-000001",        sizeof(id.serial_number) - 1);
    strncpy(id.firmware_version, "0.1.0-stub",          sizeof(id.firmware_version) - 1);
    strncpy(id.model_name,       "Thunderbird-DevKit",  sizeof(id.model_name) - 1);

    // ── Register sensor drivers ─────────────────────────────────────────
    sensor_driver_t* sensors[] = {
        &g_stub_lidar_driver,
        &g_stub_imu_driver,
        &g_stub_camera_driver,
    };
    size_t n_sensors = sizeof(sensors) / sizeof(sensors[0]);

    // ── Create transport ────────────────────────────────────────────────
    fw_transport_t* transport = fw_transport_create_tcp(port);
    if (!transport) {
        fprintf(stderr, "Failed to create TCP transport on port %u\n", port);
        return 1;
    }

    printf("Thunderbird firmware (stub) listening on port %u\n", port);
    printf("Sensors: ");
    for (size_t i = 0; i < n_sensors; ++i)
        printf("%s%s", sensors[i]->name, i + 1 < n_sensors ? ", " : "\n");

    // ── Run multiplexer (blocks until host disconnects) ─────────────────
    fw_mux_config_t cfg = fw_mux_default_config();
    int rc = fw_mux_run(sensors, n_sensors, transport, &id, &cfg);

    fw_transport_destroy(transport);

    printf("Firmware exited with code %d\n", rc);
    return rc;
}
