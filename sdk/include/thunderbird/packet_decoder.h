// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Abstract packet decoder interface
// ─────────────────────────────────────────────────────────────────────────────
//
// Generalises what PacketParser does: converts raw transport bytes into typed
// SDK sensor objects (LidarFrame, ImuSample, CameraFrame).
//
// Implementations:
//   • PacketParser           — Thunderbird native wire protocol (existing)
//   • VelodyneVlp16Decoder   — Velodyne VLP-16 / VLP-32C / Puck
//   • OusterOs1Decoder       — Ouster OS1-64 / OS1-128
//   • LivoxMid360Decoder     — Livox Mid-360
//
// Contract:
//   • feed() is called from a single I/O thread — no internal locking.
//   • Callbacks fire synchronously inside feed() on the I/O thread.
//   • Implementations must tolerate arbitrary byte boundaries (partial data).
//   • Internal parsing must not heap-allocate per packet on the hot path.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/types.h"

#include <cstddef>
#include <cstdint>
#include <string>

namespace thunderbird {

// ─── Decoder statistics ─────────────────────────────────────────────────────

struct DecoderStats {
    uint64_t packets_parsed{0};      ///< successfully decoded
    uint64_t packets_dropped{0};     ///< detected by sequence gap
    uint64_t checksum_errors{0};     ///< failed integrity check
    uint64_t resync_count{0};        ///< stream re-synchronisations
    uint64_t malformed_count{0};     ///< structurally invalid
    uint64_t bytes_processed{0};     ///< total bytes fed
};

// ─── Abstract decoder ───────────────────────────────────────────────────────

class IPacketDecoder {
public:
    virtual ~IPacketDecoder() = default;

    // ── Callback registration ───────────────────────────────────────────
    //
    // Non-virtual.  Stored in base class so all decoders share the same
    // registration surface.  ConnectionManager / HardwareDriver calls
    // these without knowing the concrete decoder type.

    void on_lidar(LidarCallback cb)    { lidar_cb_  = std::move(cb); }
    void on_imu(ImuCallback cb)        { imu_cb_    = std::move(cb); }
    void on_camera(CameraCallback cb)  { camera_cb_ = std::move(cb); }

    // ── Ingestion ───────────────────────────────────────────────────────

    /// Feed raw bytes.  Parses and dispatches complete sensor frames
    /// via the registered callbacks.  Retains partial data internally
    /// for the next call.
    virtual void feed(const uint8_t* data, size_t len) = 0;

    /// Discard internal parser state (e.g. after reconnect / link loss).
    virtual void reset() = 0;

    // ── RX timestamp injection (for SO_TIMESTAMPING) ────────────────────

    /// Set the kernel/NIC RX timestamp to be used by subsequent calls to
    /// host_timestamp().  Typical usage with SO_TIMESTAMPING is:
    ///   1) Call set_rx_timestamp() with the hardware RX time for a datagram.
    ///   2) Call feed() with the bytes from that datagram.
    /// The value persists until it is overwritten by another call to
    /// set_rx_timestamp() or cleared by passing 0.
    void set_rx_timestamp(int64_t rx_ns) { rx_timestamp_ns_ = rx_ns; }

    // ── Metadata ────────────────────────────────────────────────────────

    /// Returns a snapshot of current decoder statistics.
    /// Safe to call from any thread (returns a copy).
    virtual DecoderStats stats() const = 0;

    /// Human-readable name: "Thunderbird native", "Velodyne VLP-16", etc.
    virtual const char* decoder_name() const = 0;

protected:
    /// Returns the best available host timestamp.
    /// If SO_TIMESTAMPING injected a kernel RX timestamp, use it.
    /// Otherwise falls back to steady_clock::now().
    Timestamp host_timestamp() const {
        if (rx_timestamp_ns_ != 0)
            return Timestamp{rx_timestamp_ns_};
        return Timestamp::now();
    }

    LidarCallback  lidar_cb_;
    ImuCallback    imu_cb_;
    CameraCallback camera_cb_;

private:
    int64_t rx_timestamp_ns_{0};
};

} // namespace thunderbird
