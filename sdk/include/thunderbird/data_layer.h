// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Data Abstraction Layer
// ─────────────────────────────────────────────────────────────────────────────
//
// Converts raw sensor data (pushed by the PacketParser / simulated drivers)
// into the clean, developer-friendly types defined in sensor_data.h and
// exposes both a **Pull API** and a **Callback API** to downstream consumers.
//
// ┌──────────┐      push       ┌───────────┐   callback     ┌──────────┐
// │  Parser  │ ──────────────► │ DataLayer │ ─────────────► │  User CB │
// │ / Driver │                 │           │                └──────────┘
// └──────────┘                 │  queues   │   pull
//                              │           │ ◄───────────── user thread
//                              └───────────┘
//
// Design rationale:
//   • **Dual API**: Callback API delivers data inline on the parser thread
//     with minimal latency; Pull API decouples the producer cadence from
//     the consumer, letting the user poll at their own rate.
//   • **Independent queues**: each sensor has its own SensorQueue so a
//     slow camera consumer doesn't block the high-rate IMU path.
//   • **Conversion layer**: raw internal types (types.h / protocol.h) are
//     translated into the public `thunderbird::data::*` structs here, so
//     the public API is insulated from wire-format changes.
//   • **Zero-copy** for images: the pixel buffer is moved into a
//     shared_ptr that both the queue entry and the user callback / poll
//     result reference — no memcpy.
//   • **Thread safety**: push methods are called from producer threads;
//     pull methods from consumer threads.  The SensorQueue handles
//     synchronisation internally.
//
// Usage (Pull):
//   data::DataLayer layer;
//   // ... wire parser callbacks to layer.ingest*() ...
//   while (auto frame = layer.getNextImuFrame()) {
//       process(*frame);
//   }
//
// Usage (Callback):
//   layer.onLidarFrame([](const data::LidarFrame& f) { ... });
//   layer.onImuFrame([](const data::ImuFrame& f) { ... });
//   layer.onImageFrame([](const data::ImageFrame& f) { ... });
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/sensor_data.h"
#include "thunderbird/sensor_queue.h"
#include "thunderbird/types.h"          // internal LidarFrame, ImuSample, CameraFrame

#include <functional>
#include <memory>
#include <optional>
#include <chrono>

namespace thunderbird::data {

// ─── Callback type aliases ──────────────────────────────────────────────────

using LidarFrameCallback  = std::function<void(const LidarFrame&)>;
using ImuFrameCallback    = std::function<void(const ImuFrame&)>;
using ImageFrameCallback  = std::function<void(const ImageFrame&)>;

// ─── Queue configuration ───────────────────────────────────────────────────

/// Per-sensor queue depths.  Choose values that match the expected data rates
/// and the consumer's processing budget.  Defaults are conservative for a PoC.
struct DataLayerConfig {
    size_t lidar_queue_depth  = 64;   ///< ~6 s at 10 Hz
    size_t imu_queue_depth    = 512;  ///< ~2.5 s at 200 Hz
    size_t camera_queue_depth = 32;   ///< ~1 s at 30 Hz
};

// ─────────────────────────────────────────────────────────────────────────────

/// The Data Abstraction Layer.
///
/// Receives raw sensor data on one or more producer threads, converts to
/// the public `data::*` types, and makes them available through both
/// a callback interface and a polling (pull) interface.
class DataLayer {
public:
    explicit DataLayer(DataLayerConfig config = {})
        : lidar_q_(config.lidar_queue_depth)
        , imu_q_(config.imu_queue_depth)
        , camera_q_(config.camera_queue_depth) {}

    // ─── Pull API ───────────────────────────────────────────────────────
    //
    // Non-blocking: returns std::nullopt when the queue is empty.
    // Suitable for game-loop / render-loop style consumers.

    /// Retrieve the next LiDAR frame, or std::nullopt if none is queued.
    [[nodiscard]] std::optional<LidarFrame> getNextLidarFrame() {
        return lidar_q_.try_pop();
    }

    /// Retrieve the next IMU frame, or std::nullopt if none is queued.
    [[nodiscard]] std::optional<ImuFrame> getNextImuFrame() {
        return imu_q_.try_pop();
    }

    /// Retrieve the next camera image, or std::nullopt if none is queued.
    [[nodiscard]] std::optional<ImageFrame> getNextImageFrame() {
        return camera_q_.try_pop();
    }

    // ─── Pull API (blocking with timeout) ───────────────────────────────
    //
    // Block until a frame arrives or the timeout expires.

    template <typename Rep, typename Period>
    [[nodiscard]] std::optional<LidarFrame> waitForLidarFrame(
        std::chrono::duration<Rep, Period> timeout)
    {
        return lidar_q_.pop_for(timeout);
    }

    template <typename Rep, typename Period>
    [[nodiscard]] std::optional<ImuFrame> waitForImuFrame(
        std::chrono::duration<Rep, Period> timeout)
    {
        return imu_q_.pop_for(timeout);
    }

    template <typename Rep, typename Period>
    [[nodiscard]] std::optional<ImageFrame> waitForImageFrame(
        std::chrono::duration<Rep, Period> timeout)
    {
        return camera_q_.pop_for(timeout);
    }

    // ─── Callback API ───────────────────────────────────────────────────
    //
    // Callbacks fire **on the producer thread** (parser / driver) as soon
    // as a frame is ingested.  Keep handlers fast to avoid back-pressure
    // on the parser.  Set to nullptr to unregister.

    void onLidarFrame(LidarFrameCallback cb)  { lidar_cb_  = std::move(cb); }
    void onImuFrame(ImuFrameCallback cb)      { imu_cb_    = std::move(cb); }
    void onImageFrame(ImageFrameCallback cb)  { camera_cb_ = std::move(cb); }

    // ─── Ingestion (producer-side) ──────────────────────────────────────
    //
    // Called by the PacketParser or simulated drivers.  These convert the
    // internal SDK types into the public sensor_data types, enqueue them,
    // and invoke any registered callback.

    /// Ingest a LiDAR frame from the internal pipeline.
    void ingestLidar(std::shared_ptr<const thunderbird::LidarFrame> raw) {
        if (!raw) return;

        LidarFrame frame;
        frame.timestamp_ns = raw->timestamp.nanoseconds;
        frame.sequence     = raw->sequence_number;

        const size_t n = raw->points.size();
        frame.points.resize(n);
        for (size_t i = 0; i < n; ++i) {
            auto& dst = frame.points[i];
            auto& src = raw->points[i];
            dst.x         = src.x;
            dst.y         = src.y;
            dst.z         = src.z;
            dst.intensity = src.intensity;

            // Per-point timestamp offset: for a scan with N points evenly
            // distributed over the frame period, approximate the offset
            // as a linear interpolation.  Real hardware would provide the
            // actual per-point timing in a future protocol extension.
            if (n > 1) {
                // Assume 100 ms scan period (10 Hz) — this is a PoC default.
                // The constant can be made configurable later.
                constexpr int64_t kDefaultScanPeriodNs = 100'000'000;
                dst.timestamp_offset_ns = static_cast<int32_t>(
                    (static_cast<int64_t>(i) * kDefaultScanPeriodNs) /
                    static_cast<int64_t>(n - 1));
            } else {
                dst.timestamp_offset_ns = 0;
            }
        }

        // Enqueue for Pull API.
        lidar_q_.push(frame);

        // Fire Callback API (on producer thread).
        if (lidar_cb_) lidar_cb_(frame);
    }

    /// Ingest an IMU sample from the internal pipeline.
    void ingestImu(std::shared_ptr<const thunderbird::ImuSample> raw) {
        if (!raw) return;

        ImuFrame frame;
        frame.timestamp_ns = raw->timestamp.nanoseconds;
        frame.accel        = raw->accel;
        frame.gyro         = raw->gyro;

        imu_q_.push(frame);
        if (imu_cb_) imu_cb_(frame);
    }

    /// Ingest a camera frame from the internal pipeline.
    /// The pixel buffer is moved into shared_ptr — zero-copy for the consumer.
    void ingestCamera(std::shared_ptr<const thunderbird::CameraFrame> raw) {
        if (!raw) return;

        ImageFrame frame;
        frame.timestamp_ns = raw->timestamp.nanoseconds;
        frame.width        = raw->width;
        frame.height       = raw->height;
        frame.stride       = raw->stride;
        frame.format       = static_cast<data::PixelFormat>(
                                 static_cast<uint8_t>(raw->format));
        frame.sequence     = raw->sequence_number;

        // Zero-copy: share the existing pixel vector via shared_ptr.
        // The original CameraFrame's data vector is moved into a new
        // shared_ptr only once and then referenced by all consumers.
        frame.data = std::make_shared<const std::vector<uint8_t>>(raw->data);

        camera_q_.push(frame);
        if (camera_cb_) camera_cb_(frame);
    }

    // ─── Statistics ─────────────────────────────────────────────────────

    /// Number of LiDAR frames dropped because the queue was full.
    uint64_t lidarDropped()  const { return lidar_q_.dropped(); }
    /// Number of IMU frames dropped.
    uint64_t imuDropped()    const { return imu_q_.dropped(); }
    /// Number of camera frames dropped.
    uint64_t cameraDropped() const { return camera_q_.dropped(); }

    /// Current queue depths (pending unconsumed frames).
    size_t lidarQueued()  const { return lidar_q_.size(); }
    size_t imuQueued()    const { return imu_q_.size(); }
    size_t cameraQueued() const { return camera_q_.size(); }

    /// Drain all queues (e.g. on reconnect).
    void flush() {
        lidar_q_.clear();
        imu_q_.clear();
        camera_q_.clear();
    }

private:
    // ── Per-sensor queues ───────────────────────────────────────────────
    SensorQueue<LidarFrame>  lidar_q_;
    SensorQueue<ImuFrame>    imu_q_;
    SensorQueue<ImageFrame>  camera_q_;

    // ── User callbacks ──────────────────────────────────────────────────
    LidarFrameCallback  lidar_cb_;
    ImuFrameCallback    imu_cb_;
    ImageFrameCallback  camera_cb_;
};

} // namespace thunderbird::data
