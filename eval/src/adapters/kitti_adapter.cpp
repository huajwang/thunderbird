// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Evaluation Framework: KITTI Odometry Adapter
// ─────────────────────────────────────────────────────────────────────────────
//
// Streams KITTI Odometry benchmark data:
//
//   <sequence>/
//     velodyne/         000000.bin  000001.bin  ...
//     times.txt         one timestamp per line (seconds, float64)
//     calib.txt         optional calibration (Tr: lidar→camera0)
//   poses/
//     <seq>.txt         4×3 row-major pose per line (camera0 frame)
//
// Streaming strategy:
//   - Only ONE .bin file is loaded at a time.
//   - velodyne/ directory is scanned once at open() to build an index of
//     file paths (strings only — no point data loaded).
//   - next() reads the *next* .bin on demand, converts to PointCloudFrame,
//     then releases the previous one.
//   - Ground truth is loaded separately via loadGroundTruth() (poses are
//     small: ~4 KB for 4541 poses).
//
// KITTI .bin format:
//   Each file is N × 4 floats (x, y, z, reflectance), little-endian.
//   No header.  File size / 16 = number of points.
//
// ─────────────────────────────────────────────────────────────────────────────

#include "eval/dataset_adapter.h"
#include "eval/imu_interpolator.h"
#include "eval/perf_timer.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace eval {

// ═════════════════════════════════════════════════════════════════════════════
//  KittiAdapter — with optional synthetic IMU interpolation
// ═════════════════════════════════════════════════════════════════════════════

class KittiAdapter final : public DatasetAdapter {
public:
    /// Construct with optional IMU interpolation.
    /// If enable_imu is true and GT poses are available, synthetic IMU
    /// samples are generated between LiDAR frames using trajectory
    /// differentiation at the specified rate.
    explicit KittiAdapter(bool enable_imu = false,
                          ImuInterpolatorConfig imu_cfg = {})
        : enable_imu_(enable_imu)
        , imu_cfg_(imu_cfg)
        , interpolator_(imu_cfg) {}

    bool open(const std::string& path, size_t max_frames = 0) override;
    DatasetInfo info() const override;
    std::optional<StreamEvent> next() override;
    std::vector<GtPose> loadGroundTruth() override;
    void rewind() override;
    size_t framesEmitted() const override { return frames_emitted_; }

private:
    // ── Sequence index (built at open(), no point data) ─────────────────
    std::string              seq_path_;       // path to sequence directory
    std::string              poses_path_;     // path to poses/<seq>.txt
    std::vector<std::string> bin_paths_;      // sorted velodyne/*.bin paths
    std::vector<double>      timestamps_s_;   // times.txt values (seconds)
    size_t                   max_frames_{0};  // 0 = unlimited
    size_t                   cursor_{0};      // next .bin index to read
    size_t                   frames_emitted_{0};

    // ── IMU interpolation ───────────────────────────────────────────────
    bool                     enable_imu_{false};
    ImuInterpolatorConfig    imu_cfg_;
    ImuInterpolator          interpolator_;
    bool                     imu_ready_{false};   // true after GT loaded + prepare()
    std::vector<ImuSample>   imu_buffer_;          // pending IMU samples for current gap
    size_t                   imu_cursor_{0};       // index into imu_buffer_
    int64_t                  last_lidar_ts_ns_{0}; // timestamp of previous LiDAR frame

    // ── Pending LiDAR (held while IMU buffer drains) ────────────────────
    std::shared_ptr<const PointCloudFrame> pending_cloud_;
    int64_t                  pending_load_ns_{0};
    size_t                   pending_cursor_{0};
    bool                     has_pending_cloud_{false};

    // ── Performance timer ───────────────────────────────────────────────
    PerfTimer                timer_;

    // ── Helpers ─────────────────────────────────────────────────────────
    bool loadTimestamps(const std::string& path);
    bool scanVelodyneDir(const std::string& dir);
    std::shared_ptr<const PointCloudFrame> loadBin(size_t index);
    size_t effectiveTotal() const;
    static std::vector<GtPose> parsePoses(const std::string& path,
                                          const std::vector<double>& timestamps);
};

// ─────────────────────────────────────────────────────────────────────────────
//  open()
// ─────────────────────────────────────────────────────────────────────────────

bool KittiAdapter::open(const std::string& path, size_t max_frames) {
    seq_path_   = path;
    max_frames_ = max_frames;
    cursor_     = 0;
    frames_emitted_ = 0;
    bin_paths_.clear();
    timestamps_s_.clear();

    // ── Validate directory structure ────────────────────────────────────
    const std::string velodyne_dir = seq_path_ + "/velodyne";
    const std::string times_file   = seq_path_ + "/times.txt";

    if (!fs::is_directory(velodyne_dir)) {
        std::cerr << "[kitti] velodyne/ not found in " << seq_path_ << "\n";
        return false;
    }
    if (!fs::is_regular_file(times_file)) {
        std::cerr << "[kitti] times.txt not found in " << seq_path_ << "\n";
        return false;
    }

    // ── Build file index (paths only, no data loaded) ───────────────────
    if (!scanVelodyneDir(velodyne_dir)) return false;
    if (!loadTimestamps(times_file))    return false;

    if (bin_paths_.size() != timestamps_s_.size()) {
        std::cerr << "[kitti] mismatch: " << bin_paths_.size()
                  << " .bin files vs " << timestamps_s_.size()
                  << " timestamps\n";
        return false;
    }

    // ── Try to find poses file ──────────────────────────────────────────
    // Convention: if seq_path_ is .../sequences/00, poses at .../poses/00.txt
    const fs::path sp(seq_path_);
    const std::string seq_name = sp.filename().string();
    const fs::path poses_candidate = sp.parent_path().parent_path() / "poses" / (seq_name + ".txt");
    if (fs::is_regular_file(poses_candidate)) {
        poses_path_ = poses_candidate.string();
    }

    std::cerr << "[kitti] opened sequence: " << seq_path_
              << " (" << bin_paths_.size() << " scans";
    if (max_frames_ > 0) {
        std::cerr << ", limited to " << max_frames_;
    }
    std::cerr << ")\n";

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  info()
// ─────────────────────────────────────────────────────────────────────────────

DatasetInfo KittiAdapter::info() const {
    DatasetInfo di;
    di.name   = "kitti_" + fs::path(seq_path_).filename().string();
    di.format = "kitti";
    di.has_lidar = true;
    di.has_imu   = enable_imu_ && imu_ready_;
    di.has_ground_truth = !poses_path_.empty();

    if (!timestamps_s_.empty()) {
        di.duration_s = timestamps_s_.back() - timestamps_s_.front();
    }
    return di;
}

// ─────────────────────────────────────────────────────────────────────────────
//  next()  — stream one .bin file at a time
// ─────────────────────────────────────────────────────────────────────────────

std::optional<StreamEvent> KittiAdapter::next() {
    // ── Drain pending IMU buffer first ──────────────────────────────────
    if (imu_cursor_ < imu_buffer_.size()) {
        StreamEvent ev;
        ev.timestamp_ns = imu_buffer_[imu_cursor_].timestamp_ns;
        ev.payload      = imu_buffer_[imu_cursor_];
        ++imu_cursor_;
        return ev;
    }

    // ── Emit pending LiDAR cloud after IMU buffer is drained ────────────
    if (has_pending_cloud_) {
        StreamEvent ev;
        ev.timestamp_ns = pending_cloud_->timestamp_ns;
        ev.load_ns      = pending_load_ns_;
        ev.payload      = std::move(pending_cloud_);
        has_pending_cloud_ = false;
        pending_cloud_.reset();
        return ev;
    }

    // Check bounds + max_frames.
    if (cursor_ >= bin_paths_.size()) return std::nullopt;
    if (max_frames_ > 0 && frames_emitted_ >= max_frames_) return std::nullopt;

    // ── Load exactly one .bin ───────────────────────────────────────────
    timer_.start("load_bin");
    auto cloud = loadBin(cursor_);
    const double load_ms = timer_.stop_ms("load_bin");
    const int64_t load_ns = timer_.last_stop_ns();

    if (!cloud) {
        std::cerr << "[kitti] failed to load " << bin_paths_[cursor_] << "\n";
        ++cursor_;
        return next();  // skip corrupt files
    }

    if (frames_emitted_ % 100 == 0 || frames_emitted_ == 0) {
        std::cerr << "[kitti] frame " << frames_emitted_
                  << "/" << effectiveTotal()
                  << " | " << cloud->num_points() << " pts"
                  << " | load " << load_ms << " ms\n";
    }

    // ── Generate synthetic IMU between prev LiDAR and this one ──────────
    if (enable_imu_ && imu_ready_ && frames_emitted_ > 0) {
        imu_buffer_ = interpolator_.generateBetween(
            last_lidar_ts_ns_, cloud->timestamp_ns);
        imu_cursor_ = 0;

        // If we have buffered IMU, emit those first (before this LiDAR).
        // Re-push this LiDAR frame state so we emit it after IMU drains.
        if (!imu_buffer_.empty()) {
            // Don't advance cursor/frames yet — stash this cloud's state.
            // We'll re-enter next() after IMU buffer drains.
            last_lidar_ts_ns_ = cloud->timestamp_ns;

            // Temporarily stash the loaded cloud.
            pending_cloud_ = std::move(cloud);
            pending_load_ns_ = load_ns;
            pending_cursor_ = cursor_;
            has_pending_cloud_ = true;

            ++cursor_;
            ++frames_emitted_;

            // Start draining IMU.
            StreamEvent ev;
            ev.timestamp_ns = imu_buffer_[imu_cursor_].timestamp_ns;
            ev.payload      = imu_buffer_[imu_cursor_];
            ++imu_cursor_;
            return ev;
        }
    }

    last_lidar_ts_ns_ = cloud->timestamp_ns;

    StreamEvent ev;
    ev.timestamp_ns = cloud->timestamp_ns;
    ev.load_ns      = load_ns;
    ev.payload      = std::move(cloud);

    ++cursor_;
    ++frames_emitted_;
    return ev;
}

// ─────────────────────────────────────────────────────────────────────────────
//  loadGroundTruth()
// ─────────────────────────────────────────────────────────────────────────────

std::vector<GtPose> KittiAdapter::loadGroundTruth() {
    if (poses_path_.empty()) {
        std::cerr << "[kitti] no ground truth poses file found\n";
        return {};
    }
    auto gt = parsePoses(poses_path_, timestamps_s_);
    std::cerr << "[kitti] loaded " << gt.size() << " ground truth poses\n";

    // If IMU interpolation is enabled, prepare the interpolator now.
    if (enable_imu_ && gt.size() >= 2) {
        interpolator_.prepare(gt);
        imu_ready_ = true;
        std::cerr << "[kitti] IMU interpolation enabled at "
                  << imu_cfg_.imu_rate_hz << " Hz from "
                  << gt.size() << " GT poses\n";
    }

    return gt;
}

// ─────────────────────────────────────────────────────────────────────────────
//  rewind()
// ─────────────────────────────────────────────────────────────────────────────

void KittiAdapter::rewind() {
    cursor_         = 0;
    frames_emitted_ = 0;
    timer_.reset();
    imu_buffer_.clear();
    imu_cursor_ = 0;
    last_lidar_ts_ns_ = 0;
    has_pending_cloud_ = false;
    pending_cloud_.reset();
}

// ═════════════════════════════════════════════════════════════════════════════
//  Private helpers
// ═════════════════════════════════════════════════════════════════════════════

bool KittiAdapter::scanVelodyneDir(const std::string& dir) {
    for (const auto& entry : fs::directory_iterator(dir)) {
        if (entry.path().extension() == ".bin") {
            bin_paths_.push_back(entry.path().string());
        }
    }
    if (bin_paths_.empty()) {
        std::cerr << "[kitti] no .bin files in " << dir << "\n";
        return false;
    }
    // Sort lexicographically (000000.bin, 000001.bin, ...) for determinism.
    std::sort(bin_paths_.begin(), bin_paths_.end());
    return true;
}

bool KittiAdapter::loadTimestamps(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) return false;

    std::string line;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        timestamps_s_.push_back(std::stod(line));
    }
    return !timestamps_s_.empty();
}

/// Load a single velodyne .bin → PointCloudFrame.
/// Format: N × (float x, float y, float z, float reflectance), little-endian.
std::shared_ptr<const PointCloudFrame> KittiAdapter::loadBin(size_t index) {
    const auto& path = bin_paths_[index];
    const double ts  = timestamps_s_[index];

    // Open file and get size.
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f.is_open()) return nullptr;

    const auto file_size = f.tellg();
    if (file_size <= 0 || file_size % 16 != 0) return nullptr; // 4 floats × 4 bytes

    f.seekg(0);
    const size_t num_points = static_cast<size_t>(file_size) / 16;

    // Read raw 4-float records.
    struct KittiPoint { float x, y, z, reflectance; };
    static_assert(sizeof(KittiPoint) == 16);

    std::vector<KittiPoint> raw(num_points);
    f.read(reinterpret_cast<char*>(raw.data()),
           static_cast<std::streamsize>(file_size));

    if (!f) return nullptr;

    // Convert to PointCloudFrame.
    auto cloud = std::make_shared<PointCloudFrame>();
    // Use std::llround to avoid truncation when converting double seconds → ns.
    // double has ~15.7 significant digits; 1e9 × ts (for ts < ~1e6 s ≈ 11 days)
    // keeps full nanosecond precision.
    cloud->timestamp_ns = static_cast<int64_t>(std::llround(ts * 1.0e9));
    cloud->sequence     = static_cast<uint32_t>(index);
    cloud->is_deskewed  = false;
    cloud->points.resize(num_points);

    // Approximate per-point timestamps: assume scan spans ~100 ms.
    const double scan_period_ns = 100.0e6;  // 100 ms = 10 Hz KITTI
    for (size_t i = 0; i < num_points; ++i) {
        auto& pt  = cloud->points[i];
        pt.x      = raw[i].x;
        pt.y      = raw[i].y;
        pt.z      = raw[i].z;
        pt.intensity = raw[i].reflectance;
        pt.dt_ns  = static_cast<int32_t>(
            scan_period_ns * static_cast<double>(i) / static_cast<double>(num_points));
    }

    return cloud;
}

/// Parse KITTI poses/<seq>.txt — each line is a 3×4 row-major matrix.
/// Converts to GtPose (position + quaternion).
std::vector<GtPose> KittiAdapter::parsePoses(
    const std::string& path,
    const std::vector<double>& timestamps)
{
    std::ifstream f(path);
    if (!f.is_open()) return {};

    std::vector<GtPose> poses;
    std::string line;
    size_t idx = 0;

    while (std::getline(f, line)) {
        if (line.empty()) continue;

        // Parse 12 doubles: row-major 3×4 [ R | t ].
        std::istringstream iss(line);
        double m[12];
        for (int i = 0; i < 12; ++i) {
            if (!(iss >> m[i])) goto skip;
        }

        {
            // R is [m0 m1 m2; m3 m4 m5; m6 m7 m8], t is [m9 m10 m11]
            //   (KITTI uses row 0-2 for the upper 3x3 and col 3 for translation)
            // Actually KITTI order is: r00 r01 r02 t0 r10 r11 r12 t1 r20 r21 r22 t2
            const double r00 = m[0], r01 = m[1], r02 = m[2],  tx = m[3];
            const double r10 = m[4], r11 = m[5], r12 = m[6],  ty = m[7];
            const double r20 = m[8], r21 = m[9], r22 = m[10], tz = m[11];

            // Rotation matrix → quaternion (Shepperd's method).
            GtPose gp;
            gp.position = {tx, ty, tz};

            const double trace = r00 + r11 + r22;
            if (trace > 0.0) {
                const double s = 0.5 / std::sqrt(trace + 1.0);
                gp.quaternion[0] = 0.25 / s;                     // w
                gp.quaternion[1] = (r21 - r12) * s;              // x
                gp.quaternion[2] = (r02 - r20) * s;              // y
                gp.quaternion[3] = (r10 - r01) * s;              // z
            } else if (r00 > r11 && r00 > r22) {
                const double s = 2.0 * std::sqrt(1.0 + r00 - r11 - r22);
                gp.quaternion[0] = (r21 - r12) / s;
                gp.quaternion[1] = 0.25 * s;
                gp.quaternion[2] = (r01 + r10) / s;
                gp.quaternion[3] = (r02 + r20) / s;
            } else if (r11 > r22) {
                const double s = 2.0 * std::sqrt(1.0 + r11 - r00 - r22);
                gp.quaternion[0] = (r02 - r20) / s;
                gp.quaternion[1] = (r01 + r10) / s;
                gp.quaternion[2] = 0.25 * s;
                gp.quaternion[3] = (r12 + r21) / s;
            } else {
                const double s = 2.0 * std::sqrt(1.0 + r22 - r00 - r11);
                gp.quaternion[0] = (r10 - r01) / s;
                gp.quaternion[1] = (r02 + r20) / s;
                gp.quaternion[2] = (r12 + r21) / s;
                gp.quaternion[3] = 0.25 * s;
            }

            // Normalise quaternion.
            double norm = std::sqrt(gp.quaternion[0]*gp.quaternion[0] +
                                    gp.quaternion[1]*gp.quaternion[1] +
                                    gp.quaternion[2]*gp.quaternion[2] +
                                    gp.quaternion[3]*gp.quaternion[3]);
            if (norm > 0.0) {
                for (auto& q : gp.quaternion) q /= norm;
            }

            // Timestamp.
            if (idx < timestamps.size()) {
                gp.timestamp_ns = static_cast<int64_t>(std::llround(timestamps[idx] * 1.0e9));
            }

            poses.push_back(gp);
        }
        skip:
        ++idx;
    }

    return poses;
}

/// Effective total frame count (capped by max_frames_).
size_t KittiAdapter::effectiveTotal() const {
    const size_t total = bin_paths_.size();
    return (max_frames_ > 0 && max_frames_ < total) ? max_frames_ : total;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Factory function (called from adapter_factory.cpp)
// ═════════════════════════════════════════════════════════════════════════════

std::unique_ptr<DatasetAdapter> make_kitti_adapter(bool enable_imu,
                                                    ImuInterpolatorConfig imu_cfg) {
    return std::make_unique<KittiAdapter>(enable_imu, imu_cfg);
}

} // namespace eval
