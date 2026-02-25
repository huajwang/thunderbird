// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Player (Step 5)
// ─────────────────────────────────────────────────────────────────────────────
//
// Replays a .tbrec recording through the same API surface the live SDK uses
// (Pull API, Callback API).  This enables:
//
//   • **Offline testing** — unit & integration tests can use a canned
//     recording instead of real hardware.
//   • **Algorithm development** — researchers can iterate on perception
//     algorithms without being tethered to a device.
//   • **Demonstration** — trade-show / customer demos work without a
//     physical sensor.
//
// Design:
//
//   • The Player owns a background thread that reads records sequentially
//     and dispatches them at the original pace (or faster / slower when
//     `playback_speed` != 1.0).  Timing is maintained by sleeping between
//     records based on the *delta* between successive timestamps multiplied
//     by `1.0 / playback_speed`.
//
//   • **Pull API** — frames are pushed into internal `SensorQueue<T>`s;
//     callers poll with `getNextLidarFrame()`, `getNextImuFrame()`, etc.
//     Default queue depth is 64 (configurable).
//
//   • **Callback API** — callers register `onLidarFrame()`, `onImuFrame()`,
//     `onImageFrame()` callbacks.  They fire on the playback thread.
//
//   • When both APIs are active the same frame reaches both paths.
//
//   • Playback ends when all records are consumed.  `finished()` returns
//     `true` at that point.
//
// Usage (Pull):
//   thunderbird::data::Player player("session.tbrec");
//   player.start();
//   while (!player.finished()) {
//       if (auto f = player.getNextLidarFrame())
//           process(*f);
//       std::this_thread::sleep_for(std::chrono::milliseconds(1));
//   }
//   player.stop();
//
// Usage (Callback):
//   thunderbird::data::Player player("session.tbrec", {.playback_speed = 2.0});
//   player.onLidarFrame([](const LidarFrame& f){ process(f); });
//   player.start();
//   // ... wait for finished_ or stop() from another thread ...
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/sensor_data.h"
#include "thunderbird/sensor_queue.h"
#include "thunderbird/recording_format.h"
#include "thunderbird/recorder.h"            // RecorderDeviceInfo

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace thunderbird::data {

// ── Configuration ────────────────────────────────────────────────────────────

struct PlayerConfig {
    /// Playback speed multiplier.  1.0 = real-time; 2.0 = double speed;
    /// 0.0 or negative = as-fast-as-possible (no sleeping).
    double playback_speed{1.0};

    /// Queue depth for the pull API queues.
    std::size_t queue_depth{64};

    /// If true, loop playback when the end of the file is reached.
    bool loop{false};
};

// ── Playback statistics ──────────────────────────────────────────────────────

struct PlayerStats {
    uint64_t lidar_frames{0};
    uint64_t imu_frames{0};
    uint64_t camera_frames{0};
    uint64_t total_records{0};
};

// ── Player class ─────────────────────────────────────────────────────────────

class Player {
public:
    // Callback types — fire on the playback thread.
    using LidarCb  = std::function<void(const LidarFrame&)>;
    using ImuCb    = std::function<void(const ImuFrame&)>;
    using ImageCb  = std::function<void(const ImageFrame&)>;

    /// Construct a Player for the given .tbrec file.
    explicit Player(const std::string& file_path,
                    PlayerConfig config = {});

    ~Player();

    // Non-copyable, non-movable.
    Player(const Player&) = delete;
    Player& operator=(const Player&) = delete;

    // ── Lifecycle ───────────────────────────────────────────────────────

    /// Open the file, validate the header, and start the playback thread.
    bool start();

    /// Signal the playback thread to stop and join it.
    void stop();

    /// True once `start()` has been called and playback hasn't finished/stopped.
    [[nodiscard]] bool playing() const noexcept {
        return playing_.load(std::memory_order_relaxed);
    }

    /// True when all records have been dispatched (and loop == false).
    [[nodiscard]] bool finished() const noexcept {
        return finished_.load(std::memory_order_relaxed);
    }

    // ── Pull API ────────────────────────────────────────────────────────

    [[nodiscard]] std::optional<LidarFrame>  getNextLidarFrame();
    [[nodiscard]] std::optional<ImuFrame>    getNextImuFrame();
    [[nodiscard]] std::optional<ImageFrame>  getNextImageFrame();

    // ── Callback API ────────────────────────────────────────────────────

    void onLidarFrame(LidarCb cb);
    void onImuFrame(ImuCb cb);
    void onImageFrame(ImageCb cb);

    // ── Metadata ────────────────────────────────────────────────────────

    /// Device info extracted from the file header (available after start()).
    [[nodiscard]] RecorderDeviceInfo deviceInfo() const;

    /// Statistics (available any time after start()).
    [[nodiscard]] PlayerStats stats() const noexcept;

private:
    void playback_loop();

    bool read_lidar_record(const recording::RecordHeader& hdr);
    bool read_imu_record(const recording::RecordHeader& hdr);
    bool read_camera_record(const recording::RecordHeader& hdr);

    void dispatch_lidar(const LidarFrame& f);
    void dispatch_imu(const ImuFrame& f);
    void dispatch_camera(const ImageFrame& f);

    // ── Data members ────────────────────────────────────────────────────

    std::string    file_path_;
    PlayerConfig   config_;
    std::FILE*     fp_{nullptr};
    std::thread    thread_;

    std::atomic<bool> playing_{false};
    std::atomic<bool> finished_{false};
    std::atomic<bool> stop_requested_{false};

    // Queues for pull API
    SensorQueue<LidarFrame>  lidar_queue_;
    SensorQueue<ImuFrame>    imu_queue_;
    SensorQueue<ImageFrame>  image_queue_;

    // Callbacks (set before start(); read from playback thread)
    LidarCb  lidar_cb_;
    ImuCb    imu_cb_;
    ImageCb  image_cb_;

    // Metadata from file header
    RecorderDeviceInfo  device_info_;
    mutable std::mutex  info_mtx_;

    // Stats (updated from playback thread, read from any thread)
    mutable std::mutex  stats_mtx_;
    PlayerStats         stats_;
};

} // namespace thunderbird::data
