// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Hardware sensor driver (transport-backed)
// ─────────────────────────────────────────────────────────────────────────────
//
// Unlike the per-sensor simulated drivers (which each own a thread and
// generate data), the hardware driver is a single unified driver that:
//   1. Owns a ConnectionManager (transport + parser + state machine).
//   2. Registers per-sensor callbacks on the PacketParser.
//   3. Delegates streaming start/stop to the device via the wire protocol.
//
// This collapses three ISensorDriver instances into one HardwareDriver that
// manages the entire device.  The DeviceManager adapts it by wrapping three
// thin ISensorDriver facades (see HardwareSensorFacade below).
//
// Design choices:
//   • Single ConnectionManager per device rather than three transports.
//     Hardware multiplexes all sensor channels over one link.
//   • Callbacks are forwarded from the parser's I/O thread — zero extra
//     thread hops for raw data.
//   • The start/stop protocol is at the device level, not per-sensor.
//     Individual sensor enable/disable can be added later via a bitmask
//     in the StartStream payload.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/connection_manager.h"
#include "thunderbird/sensor_driver.h"
#include "thunderbird/types.h"

#include <atomic>
#include <memory>
#include <string>

namespace thunderbird {

// ─── HardwareDriver ─────────────────────────────────────────────────────────

class HardwareDriver {
public:
    /// Construct with an already-configured ConnectionManager.
    explicit HardwareDriver(std::unique_ptr<ConnectionManager> conn_mgr)
        : conn_(std::move(conn_mgr)) {}

    // ── Callback registration (must be done before connect) ─────────────────

    void set_lidar_callback(LidarCallback cb) {
        conn_->parser().on_lidar(std::move(cb));
    }

    void set_imu_callback(ImuCallback cb) {
        conn_->parser().on_imu(std::move(cb));
    }

    void set_camera_callback(CameraCallback cb) {
        conn_->parser().on_camera(std::move(cb));
    }

    // ── Lifecycle ───────────────────────────────────────────────────────────

    Status connect(const std::string& uri) {
        return conn_->connect(uri);
    }

    Status start_streaming() {
        if (streaming_) return Status::AlreadyStreaming;
        Status s = conn_->start_streaming();
        if (s == Status::OK) streaming_ = true;
        return s;
    }

    Status stop_streaming() {
        if (!streaming_) return Status::OK;
        Status s = conn_->stop_streaming();
        streaming_ = false;
        return s;
    }

    void disconnect() {
        streaming_ = false;
        conn_->disconnect();
    }

    bool is_streaming() const { return streaming_; }

    DeviceInfo device_info() const { return conn_->device_info(); }

    ConnectionManager& connection() { return *conn_; }

private:
    std::unique_ptr<ConnectionManager> conn_;
    std::atomic<bool>                  streaming_{false};
};

// ─── ISensorDriver facade ───────────────────────────────────────────────────

/// Thin adapter so the DeviceManager can treat each sensor channel as if it
/// had its own ISensorDriver (matches the simulated-driver interface).
/// All three facades share the same underlying HardwareDriver.
class HardwareSensorFacade final : public ISensorDriver {
public:
    HardwareSensorFacade(SensorType type, std::shared_ptr<HardwareDriver> hw)
        : type_(type), hw_(std::move(hw)) {}

    SensorType  type() const override { return type_; }
    std::string name() const override {
        return std::string("HW-") + sensor_type_name(type_);
    }

    /// Initialization is handled at the HardwareDriver level; this is a no-op.
    Status initialize() override { return Status::OK; }

    /// Start is handled at the HardwareDriver level — only the first facade
    /// to call actually triggers the device.
    Status start_streaming() override {
        return hw_->start_streaming();
    }

    Status stop_streaming() override {
        return hw_->stop_streaming();
    }

    bool is_streaming() const override { return hw_->is_streaming(); }

private:
    SensorType                     type_;
    std::shared_ptr<HardwareDriver> hw_;
};

} // namespace thunderbird
