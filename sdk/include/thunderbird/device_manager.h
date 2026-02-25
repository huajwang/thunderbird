// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Public API: DeviceManager
// ─────────────────────────────────────────────────────────────────────────────
//
// This is the primary entry-point for end-users.  It owns the sensor drivers,
// the time-sync engine, and exposes simple register-callback / start / stop.
//
// Usage:
//   thunderbird::DeviceManager dev;
//   dev.on_lidar([](auto frame) { /* process */ });
//   dev.on_sync([](auto bundle) { /* fused data */ });
//   dev.connect();
//   dev.start();
//   // ... run ...
//   dev.stop();
//   dev.disconnect();
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/types.h"
#include "thunderbird/sync_engine.h"
#include "thunderbird/sensor_driver.h"
#include "thunderbird/connection.h"
#include "thunderbird/connection_manager.h"
#include "thunderbird/data_layer.h"
#include "thunderbird/time_sync.h"

#include <memory>
#include <string>
#include <vector>
#include <functional>

namespace thunderbird {

/// Configuration handed to DeviceManager at construction.
struct DeviceConfig {
    /// Connection URI.  For simulated mode an empty string is fine.
    /// Hardware: "eth://192.168.1.100:7500" or "usb://0"
    std::string uri;

    /// Simulated-sensor rates (only used when THUNDERBIRD_SIMULATED is defined).
    double lidar_hz  = 10.0;
    double imu_hz    = 200.0;
    double camera_fps = 30.0;
    uint32_t camera_width  = 640;
    uint32_t camera_height = 480;

    /// Sync engine configuration (Phase 1 — legacy).
    SyncConfig sync;

    /// Time-synchronization layer (Step 3 — SyncedFrame API).
    data::TimeSyncConfig time_sync;

    /// Connection parameters (timeouts, heartbeat, client id).
    ConnectionConfig connection;

    /// Retry / reconnect policy.
    RetryConfig retry;
};

class DeviceManager {
public:
    explicit DeviceManager(DeviceConfig config = {});
    ~DeviceManager();

    // non-copyable
    DeviceManager(const DeviceManager&) = delete;
    DeviceManager& operator=(const DeviceManager&) = delete;

    // ── Connection ──────────────────────────────────────────────────────────

    /// Establish connection to the device (or create simulated backends).
    Status connect();

    /// Gracefully disconnect.
    Status disconnect();

    bool is_connected() const;

    /// Retrieve static device info (serial, FW version, model).
    DeviceInfo device_info() const;

    // ── Streaming ───────────────────────────────────────────────────────────

    /// Start all sensor streams + sync engine.
    Status start();

    /// Stop all streams.
    Status stop();

    bool is_streaming() const;

    // ── Raw-data callbacks ──────────────────────────────────────────────────

    void on_lidar(LidarCallback cb);
    void on_imu(ImuCallback cb);
    void on_camera(CameraCallback cb);

    // ── Synchronized bundle callback (Phase 1 legacy) ───────────────────────

    void on_sync(SyncCallback cb);

    // ── Time Synchronization Layer (Step 3) ──────────────────────────────────

    /// Access the time-sync engine for Pull / Callback / Stats API.
    data::TimeSyncEngine& time_sync();
    const data::TimeSyncEngine& time_sync() const;

    /// Convenience: register a SyncedFrame callback.
    void on_synced_frame(data::SyncedFrameCallback cb);

    // ── Data Abstraction Layer ───────────────────────────────────────────────

    /// Access the Data Abstraction Layer for Pull API / Callback API.
    /// Returns a non-owning reference; lifetime is tied to the DeviceManager.
    data::DataLayer& data_layer();
    const data::DataLayer& data_layer() const;

    // ── Statistics ──────────────────────────────────────────────────────────

    uint64_t sync_bundles_produced() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace thunderbird
