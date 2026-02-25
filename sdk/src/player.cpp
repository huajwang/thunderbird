// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Player implementation
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/player.h"

#include <algorithm>
#include <cstring>

namespace thunderbird::data {

// ─── Constructor / Destructor ───────────────────────────────────────────────

Player::Player(const std::string& file_path, PlayerConfig config)
    : file_path_(file_path)
    , config_(config)
    , lidar_queue_(config.queue_depth)
    , imu_queue_(config.queue_depth)
    , image_queue_(config.queue_depth)
{}

Player::~Player() {
    if (playing()) stop();
}

// ─── Lifecycle ──────────────────────────────────────────────────────────────

bool Player::start() {
    if (playing()) return false;

    fp_ = std::fopen(file_path_.c_str(), "rb");
    if (!fp_) return false;

    // Read & validate file header.
    using namespace thunderbird::recording;

    FileHeader fh{};
    if (std::fread(&fh, sizeof(fh), 1, fp_) != 1) {
        std::fclose(fp_);
        fp_ = nullptr;
        return false;
    }

    if (std::memcmp(fh.magic, kFileMagic, 8) != 0 ||
        fh.version != kFormatVersion)
    {
        std::fclose(fp_);
        fp_ = nullptr;
        return false;
    }

    // Extract device info.
    {
        std::lock_guard lock(info_mtx_);
        device_info_.serial_number   = std::string(fh.serial_number,
            strnlen(fh.serial_number, sizeof(fh.serial_number)));
        device_info_.firmware_version = std::string(fh.firmware_version,
            strnlen(fh.firmware_version, sizeof(fh.firmware_version)));
        device_info_.model_name      = std::string(fh.model_name,
            strnlen(fh.model_name, sizeof(fh.model_name)));
    }

    // Clear state.
    {
        std::lock_guard lock(stats_mtx_);
        stats_ = {};
    }
    lidar_queue_.clear();
    imu_queue_.clear();
    image_queue_.clear();
    finished_.store(false, std::memory_order_release);
    stop_requested_.store(false, std::memory_order_release);
    playing_.store(true, std::memory_order_release);

    // Launch playback thread.
    thread_ = std::thread(&Player::playback_loop, this);
    return true;
}

void Player::stop() {
    stop_requested_.store(true, std::memory_order_release);
    if (thread_.joinable()) thread_.join();
    playing_.store(false, std::memory_order_release);

    if (fp_) {
        std::fclose(fp_);
        fp_ = nullptr;
    }
}

// ─── Pull API ───────────────────────────────────────────────────────────────

std::optional<LidarFrame> Player::getNextLidarFrame() {
    return lidar_queue_.try_pop();
}

std::optional<ImuFrame> Player::getNextImuFrame() {
    return imu_queue_.try_pop();
}

std::optional<ImageFrame> Player::getNextImageFrame() {
    return image_queue_.try_pop();
}

// ─── Callback API ───────────────────────────────────────────────────────────

void Player::onLidarFrame(LidarCb cb)  { lidar_cb_ = std::move(cb); }
void Player::onImuFrame(ImuCb cb)      { imu_cb_   = std::move(cb); }
void Player::onImageFrame(ImageCb cb)  { image_cb_  = std::move(cb); }

// ─── Metadata / Stats ───────────────────────────────────────────────────────

RecorderDeviceInfo Player::deviceInfo() const {
    std::lock_guard lock(info_mtx_);
    return device_info_;
}

PlayerStats Player::stats() const noexcept {
    std::lock_guard lock(stats_mtx_);
    return stats_;
}

// ─── Playback thread ────────────────────────────────────────────────────────

void Player::playback_loop() {
    using namespace thunderbird::recording;
    namespace sc = std::chrono;

    int64_t prev_ts = 0;               // previous record's timestamp
    bool first = true;

    auto wall_ref = sc::steady_clock::now();   // wall-clock reference

    while (!stop_requested_.load(std::memory_order_acquire)) {
        RecordHeader rhdr{};
        if (std::fread(&rhdr, sizeof(rhdr), 1, fp_) != 1) {
            // End of file — possibly loop.
            if (config_.loop) {
                std::fseek(fp_,
                           static_cast<long>(sizeof(FileHeader)),
                           SEEK_SET);
                first = true;
                continue;
            }
            break;   // done
        }

        // ── Pace control ────────────────────────────────────────────────
        if (config_.playback_speed > 0.0) {
            if (first) {
                prev_ts  = rhdr.timestamp_ns;
                wall_ref = sc::steady_clock::now();
                first    = false;
            } else {
                const int64_t delta_ns = rhdr.timestamp_ns - prev_ts;
                if (delta_ns > 0) {
                    const auto target = wall_ref
                        + sc::nanoseconds(
                            static_cast<int64_t>(delta_ns / config_.playback_speed));
                    std::this_thread::sleep_until(target);
                }
                wall_ref = sc::steady_clock::now();
                prev_ts  = rhdr.timestamp_ns;
            }
        }

        // ── Dispatch by sensor type ─────────────────────────────────────
        bool ok = false;
        switch (rhdr.sensor) {
            case SensorTag::LiDAR:  ok = read_lidar_record(rhdr);  break;
            case SensorTag::IMU:    ok = read_imu_record(rhdr);    break;
            case SensorTag::Camera: ok = read_camera_record(rhdr); break;
            default:
                // Unknown sensor — skip payload.
                std::fseek(fp_, rhdr.payload_size, SEEK_CUR);
                ok = true;
                break;
        }
        if (!ok) break;  // read error / corruption
    }

    finished_.store(true, std::memory_order_release);
    playing_.store(false, std::memory_order_release);
}

// ─── Record readers ─────────────────────────────────────────────────────────

bool Player::read_lidar_record(const recording::RecordHeader& hdr) {
    using namespace thunderbird::recording;

    LidarRecordHeader lhdr{};
    if (std::fread(&lhdr, sizeof(lhdr), 1, fp_) != 1) return false;

    LidarFrame frame;
    frame.timestamp_ns = hdr.timestamp_ns;
    frame.sequence     = lhdr.sequence;
    frame.points.resize(lhdr.point_count);

    static_assert(sizeof(PointXYZIT) == sizeof(PointRecord));
    if (lhdr.point_count > 0) {
        if (std::fread(frame.points.data(), sizeof(PointRecord),
                       lhdr.point_count, fp_) != lhdr.point_count)
            return false;
    }

    dispatch_lidar(frame);

    {
        std::lock_guard lock(stats_mtx_);
        ++stats_.lidar_frames;
        ++stats_.total_records;
    }
    return true;
}

bool Player::read_imu_record(const recording::RecordHeader& hdr) {
    using namespace thunderbird::recording;

    ImuRecordPayload ipay{};
    if (std::fread(&ipay, sizeof(ipay), 1, fp_) != 1) return false;

    ImuFrame frame;
    frame.timestamp_ns = hdr.timestamp_ns;
    frame.accel[0] = ipay.accel_x;
    frame.accel[1] = ipay.accel_y;
    frame.accel[2] = ipay.accel_z;
    frame.gyro[0]  = ipay.gyro_x;
    frame.gyro[1]  = ipay.gyro_y;
    frame.gyro[2]  = ipay.gyro_z;

    dispatch_imu(frame);

    {
        std::lock_guard lock(stats_mtx_);
        ++stats_.imu_frames;
        ++stats_.total_records;
    }
    return true;
}

bool Player::read_camera_record(const recording::RecordHeader& hdr) {
    using namespace thunderbird::recording;

    CameraRecordHeader chdr{};
    if (std::fread(&chdr, sizeof(chdr), 1, fp_) != 1) return false;

    ImageFrame frame;
    frame.timestamp_ns = hdr.timestamp_ns;
    frame.width        = chdr.width;
    frame.height       = chdr.height;
    frame.stride       = chdr.stride;
    frame.format       = static_cast<PixelFormat>(chdr.pixel_format);
    frame.sequence     = chdr.sequence;

    if (chdr.pixel_data_size > 0) {
        auto buf = std::make_shared<std::vector<uint8_t>>(chdr.pixel_data_size);
        if (std::fread(buf->data(), 1, chdr.pixel_data_size, fp_)
            != chdr.pixel_data_size)
            return false;
        frame.data = std::move(buf);
    }

    dispatch_camera(frame);

    {
        std::lock_guard lock(stats_mtx_);
        ++stats_.camera_frames;
        ++stats_.total_records;
    }
    return true;
}

// ─── Dispatch helpers ───────────────────────────────────────────────────────

void Player::dispatch_lidar(const LidarFrame& f) {
    lidar_queue_.push(f);
    if (lidar_cb_) lidar_cb_(f);
}

void Player::dispatch_imu(const ImuFrame& f) {
    imu_queue_.push(f);
    if (imu_cb_) imu_cb_(f);
}

void Player::dispatch_camera(const ImageFrame& f) {
    image_queue_.push(f);
    if (image_cb_) image_cb_(f);
}

} // namespace thunderbird::data
