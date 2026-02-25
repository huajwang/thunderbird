// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Recorder implementation
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/recorder.h"

#include <algorithm>
#include <cstring>

namespace thunderbird::data {

// ─── Helpers ────────────────────────────────────────────────────────────────

namespace {

/// Copy a std::string into a fixed-size char buffer, null-terminating.
void copy_fixed(char* dst, std::size_t dst_size, const std::string& src) {
    std::memset(dst, 0, dst_size);
    const auto n = std::min(src.size(), dst_size - 1);
    std::memcpy(dst, src.data(), n);
}

} // anonymous namespace

// ─── Constructor / Destructor ───────────────────────────────────────────────

Recorder::Recorder(const std::string& file_path, RecorderDeviceInfo info)
    : file_path_(file_path)
    , device_info_(std::move(info))
{}

Recorder::~Recorder() {
    if (recording()) stop();
}

// ─── Lifecycle ──────────────────────────────────────────────────────────────

bool Recorder::start() {
    std::lock_guard lock(mtx_);
    if (recording_) return false;           // already recording

    fp_ = std::fopen(file_path_.c_str(), "wb");
    if (!fp_) return false;

    // Large user-space buffer to amortise syscalls.
    std::setvbuf(fp_, nullptr, _IOFBF, 256 * 1024);

    first_ts_     = 0;
    last_ts_      = 0;
    record_count_ = 0;
    stats_        = {};

    write_header();                         // placeholder header (counts = 0)
    recording_.store(true, std::memory_order_release);
    return true;
}

void Recorder::stop() {
    std::lock_guard lock(mtx_);
    if (!recording_) return;
    recording_.store(false, std::memory_order_release);

    finalise_header();
    std::fclose(fp_);
    fp_ = nullptr;
}

// ─── Frame recording ────────────────────────────────────────────────────────

void Recorder::recordLidarFrame(const LidarFrame& frame) {
    if (!recording()) return;

    using namespace thunderbird::recording;

    LidarRecordHeader lhdr{};
    lhdr.sequence    = frame.sequence;
    lhdr.point_count = static_cast<uint32_t>(frame.points.size());

    const uint32_t pts_bytes = lhdr.point_count * sizeof(PointRecord);
    const uint32_t payload   = sizeof(LidarRecordHeader) + pts_bytes;

    RecordHeader rhdr{};
    rhdr.sensor       = SensorTag::LiDAR;
    rhdr.reserved[0]  = rhdr.reserved[1] = rhdr.reserved[2] = 0;
    rhdr.timestamp_ns = frame.timestamp_ns;
    rhdr.payload_size = payload;

    std::lock_guard lock(mtx_);
    if (!fp_) return;

    std::fwrite(&rhdr, sizeof(rhdr), 1, fp_);
    std::fwrite(&lhdr, sizeof(lhdr), 1, fp_);

    // Write points — PointXYZIT and PointRecord have identical layout.
    static_assert(sizeof(PointXYZIT) == sizeof(PointRecord));
    if (!frame.points.empty()) {
        std::fwrite(frame.points.data(), sizeof(PointRecord),
                    frame.points.size(), fp_);
    }

    // Bookkeeping
    if (record_count_ == 0) first_ts_ = frame.timestamp_ns;
    last_ts_ = frame.timestamp_ns;
    ++record_count_;
    ++stats_.lidar_frames;
    stats_.total_bytes += sizeof(rhdr) + payload;
}

void Recorder::recordImuFrame(const ImuFrame& frame) {
    if (!recording()) return;

    using namespace thunderbird::recording;

    ImuRecordPayload ipay{};
    ipay.accel_x = frame.accel[0];
    ipay.accel_y = frame.accel[1];
    ipay.accel_z = frame.accel[2];
    ipay.gyro_x  = frame.gyro[0];
    ipay.gyro_y  = frame.gyro[1];
    ipay.gyro_z  = frame.gyro[2];

    RecordHeader rhdr{};
    rhdr.sensor       = SensorTag::IMU;
    rhdr.reserved[0]  = rhdr.reserved[1] = rhdr.reserved[2] = 0;
    rhdr.timestamp_ns = frame.timestamp_ns;
    rhdr.payload_size = sizeof(ImuRecordPayload);

    std::lock_guard lock(mtx_);
    if (!fp_) return;

    std::fwrite(&rhdr, sizeof(rhdr), 1, fp_);
    std::fwrite(&ipay, sizeof(ipay), 1, fp_);

    if (record_count_ == 0) first_ts_ = frame.timestamp_ns;
    last_ts_ = frame.timestamp_ns;
    ++record_count_;
    ++stats_.imu_frames;
    stats_.total_bytes += sizeof(rhdr) + sizeof(ipay);
}

void Recorder::recordImageFrame(const ImageFrame& frame) {
    if (!recording()) return;

    using namespace thunderbird::recording;

    const uint32_t pix_size = frame.data
        ? static_cast<uint32_t>(frame.data->size())
        : 0u;

    CameraRecordHeader chdr{};
    chdr.width           = frame.width;
    chdr.height          = frame.height;
    chdr.stride          = frame.stride;
    chdr.pixel_format    = static_cast<uint8_t>(frame.format);
    chdr.reserved[0]     = chdr.reserved[1] = chdr.reserved[2] = 0;
    chdr.sequence        = frame.sequence;
    chdr.pixel_data_size = pix_size;

    const uint32_t payload = sizeof(CameraRecordHeader) + pix_size;

    RecordHeader rhdr{};
    rhdr.sensor       = SensorTag::Camera;
    rhdr.reserved[0]  = rhdr.reserved[1] = rhdr.reserved[2] = 0;
    rhdr.timestamp_ns = frame.timestamp_ns;
    rhdr.payload_size = payload;

    std::lock_guard lock(mtx_);
    if (!fp_) return;

    std::fwrite(&rhdr, sizeof(rhdr), 1, fp_);
    std::fwrite(&chdr, sizeof(chdr), 1, fp_);
    if (pix_size > 0 && frame.data) {
        std::fwrite(frame.data->data(), 1, pix_size, fp_);
    }

    if (record_count_ == 0) first_ts_ = frame.timestamp_ns;
    last_ts_ = frame.timestamp_ns;
    ++record_count_;
    ++stats_.camera_frames;
    stats_.total_bytes += sizeof(rhdr) + payload;
}

// ─── Statistics ─────────────────────────────────────────────────────────────

RecorderStats Recorder::stats() const noexcept {
    std::lock_guard lock(mtx_);
    return stats_;
}

// ─── Private helpers ────────────────────────────────────────────────────────

void Recorder::write_header() {
    using namespace thunderbird::recording;

    FileHeader fh{};
    std::memcpy(fh.magic, kFileMagic, 8);
    fh.version = kFormatVersion;
    fh.flags   = 0;

    copy_fixed(fh.serial_number, sizeof(fh.serial_number),
               device_info_.serial_number);
    copy_fixed(fh.firmware_version, sizeof(fh.firmware_version),
               device_info_.firmware_version);
    copy_fixed(fh.model_name, sizeof(fh.model_name),
               device_info_.model_name);

    fh.start_timestamp_ns = 0;
    fh.stop_timestamp_ns  = 0;
    fh.total_records      = 0;

    std::fwrite(&fh, sizeof(fh), 1, fp_);
}

void Recorder::finalise_header() {
    using namespace thunderbird::recording;

    if (!fp_) return;

    // Seek back to the start and rewrite the header with final values.
    std::fseek(fp_, 0, SEEK_SET);

    FileHeader fh{};
    std::memcpy(fh.magic, kFileMagic, 8);
    fh.version = kFormatVersion;
    fh.flags   = 0;

    copy_fixed(fh.serial_number, sizeof(fh.serial_number),
               device_info_.serial_number);
    copy_fixed(fh.firmware_version, sizeof(fh.firmware_version),
               device_info_.firmware_version);
    copy_fixed(fh.model_name, sizeof(fh.model_name),
               device_info_.model_name);

    fh.start_timestamp_ns = first_ts_;
    fh.stop_timestamp_ns  = last_ts_;
    fh.total_records      = record_count_;

    std::fwrite(&fh, sizeof(fh), 1, fp_);
    std::fflush(fp_);
}

} // namespace thunderbird::data
