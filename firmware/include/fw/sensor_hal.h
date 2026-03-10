// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird Firmware — Sensor HAL (Hardware Abstraction Layer)
// ─────────────────────────────────────────────────────────────────────────────
//
// Chip-agnostic interface for sensor peripherals.  Each sensor type (LiDAR,
// IMU, camera) implements this interface.  The multiplexer polls sensors
// through this uniform API without knowing the underlying bus or chip.
//
// Swap the stub drivers for real chip drivers once hardware is selected.
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ─── Sensor types ───────────────────────────────────────────────────────────

typedef enum {
    SENSOR_LIDAR  = 0,
    SENSOR_IMU    = 1,
    SENSOR_CAMERA = 2,
    SENSOR_COUNT  = 3,
} sensor_type_t;

// ─── Sensor driver vtable ───────────────────────────────────────────────────

/// Each sensor driver provides these three operations.
/// Implementations must be safe to call from the main firmware loop.
typedef struct sensor_driver {
    /// Human-readable name for diagnostics ("stub-lidar", "icm42688", …).
    const char* name;

    /// Sensor type tag.
    sensor_type_t type;

    /// Initialise the sensor peripheral.
    /// Returns 0 on success, negative errno-style code on failure.
    int (*init)(struct sensor_driver* self);

    /// Read one sample / frame into `buf`.
    /// @param buf           Output buffer for sensor-specific payload.
    /// @param max_len       Size of buf in bytes.
    /// @param timestamp_ns  [out] Device-clock timestamp for this reading.
    /// @returns             Bytes written to buf, or 0 if no data ready,
    ///                      or negative on error.
    int (*read)(struct sensor_driver* self,
                uint8_t* buf, size_t max_len,
                uint64_t* timestamp_ns);

    /// Shut down the sensor peripheral gracefully.
    void (*shutdown)(struct sensor_driver* self);

    /// Opaque pointer for driver-private state.
    void* priv;
} sensor_driver_t;

#ifdef __cplusplus
}
#endif
