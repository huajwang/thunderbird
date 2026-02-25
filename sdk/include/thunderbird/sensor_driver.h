// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Abstract sensor driver interface
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/types.h"
#include <atomic>
#include <string>

namespace thunderbird {

/// Base interface for each sensor driver (LiDAR, IMU, Camera).
/// Drivers own their own I/O thread and push data via registered callbacks.
class ISensorDriver {
public:
    virtual ~ISensorDriver() = default;

    virtual SensorType type() const = 0;
    virtual std::string name() const = 0;

    /// Initialise the driver (open resources, validate firmware).
    virtual Status initialize() = 0;

    /// Begin streaming on an internal thread.
    virtual Status start_streaming() = 0;

    /// Stop streaming (blocks until the internal thread joins).
    virtual Status stop_streaming() = 0;

    /// Returns true while streaming is active.
    virtual bool is_streaming() const = 0;
};

} // namespace thunderbird
