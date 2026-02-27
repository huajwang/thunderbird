// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Evaluation Framework: EuRoC MAV Adapter
// ─────────────────────────────────────────────────────────────────────────────
//
// Streams EuRoC Micro Aerial Vehicle (MAV) benchmark data:
//
//   <sequence>/
//     mav0/
//       imu0/
//         data.csv         timestamp[ns],wx,wy,wz,ax,ay,az
//       cam0/...           (not used — LiDAR-inertial only)
//       leica0/ or vicon0/ or state_groundtruth/
//         data.csv         timestamp[ns],px,py,pz,qw,qx,qy,qz,...
//
// For LiDAR evaluation on EuRoC, we synthesise point clouds from the
// ground truth trajectory (the dataset is camera+IMU, but we use it to
// stress-test the IMU integration + state estimation path).
//
// If a pointcloud_0/ directory exists (e.g. from EuRoC extensions or
// user-provided LiDAR data), it is loaded directly.
//
// ─────────────────────────────────────────────────────────────────────────────

#include "eval/dataset_adapter.h"
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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace fs = std::filesystem;

namespace eval {

// ═════════════════════════════════════════════════════════════════════════════
//  EurocAdapter
// ═════════════════════════════════════════════════════════════════════════════

class EurocAdapter final : public DatasetAdapter {
public:
    bool open(const std::string& path, size_t max_frames = 0) override;
    DatasetInfo info() const override;
    std::optional<StreamEvent> next() override;
    std::vector<GtPose> loadGroundTruth() override;
    void rewind() override;
    size_t framesEmitted() const override { return frames_emitted_; }

private:
    // ── Sequence data ───────────────────────────────────────────────────
    std::string seq_path_;
    size_t      max_frames_{0};
    size_t      frames_emitted_{0};

    // ── IMU ─────────────────────────────────────────────────────────────
    struct ImuRow {
        int64_t ts_ns;
        double  wx, wy, wz;   // gyro (rad/s)
        double  ax, ay, az;   // accel (m/s²)
    };
    std::vector<ImuRow> imu_rows_;
    size_t imu_cursor_{0};

    // ── Ground truth ────────────────────────────────────────────────────
    struct GtRow {
        int64_t ts_ns;
        double  px, py, pz;
        double  qw, qx, qy, qz;
    };
    std::vector<GtRow> gt_rows_;

    // ── Synthetic LiDAR from GT ─────────────────────────────────────────
    // Generate a cloud at ~10 Hz from GT poses (one cloud per 100 ms).
    std::vector<int64_t> lidar_timestamps_ns_;
    size_t lidar_cursor_{0};

    // ── Unified timeline ────────────────────────────────────────────────
    // Interleaved events are served from a merged timeline.
    struct TimelineEvent {
        int64_t ts_ns;
        enum Type { IMU, LIDAR } type;
        size_t  index;
    };
    std::vector<TimelineEvent> timeline_;
    size_t timeline_cursor_{0};

    PerfTimer timer_;

    // ── Helpers ─────────────────────────────────────────────────────────
    bool loadImu(const std::string& csv_path);
    bool loadGt(const std::string& csv_path);
    void buildTimeline();
    std::shared_ptr<const PointCloudFrame> synthCloud(size_t lidar_idx);

    /// Quaternion rotation: q ⊗ [0,v] ⊗ q*
    static std::array<double,3> rotateByQuat(
        double qw, double qx, double qy, double qz,
        double vx, double vy, double vz);
};

// ─────────────────────────────────────────────────────────────────────────────
//  open()
// ─────────────────────────────────────────────────────────────────────────────

bool EurocAdapter::open(const std::string& path, size_t max_frames) {
    seq_path_   = path;
    max_frames_ = max_frames;
    rewind();

    // ── Locate mav0/ subdirectory ───────────────────────────────────────
    fs::path base(path);
    fs::path mav0 = base;
    if (fs::is_directory(base / "mav0")) {
        mav0 = base / "mav0";
    }

    // ── Load IMU data ───────────────────────────────────────────────────
    fs::path imu_csv;
    for (auto& candidate : {
        mav0 / "imu0" / "data.csv",
        base / "imu0" / "data.csv",
    }) {
        if (fs::is_regular_file(candidate)) {
            imu_csv = candidate;
            break;
        }
    }
    if (imu_csv.empty() || !loadImu(imu_csv.string())) {
        std::cerr << "[euroc] imu0/data.csv not found in " << path << "\n";
        return false;
    }

    // ── Load ground truth ───────────────────────────────────────────────
    fs::path gt_csv;
    for (auto& candidate : {
        mav0 / "state_groundtruth_estimate0" / "data.csv",
        mav0 / "leica0" / "data.csv",
        mav0 / "vicon0" / "data.csv",
        mav0 / "ground_truth" / "data.csv",
        base / "state_groundtruth_estimate0" / "data.csv",
    }) {
        if (fs::is_regular_file(candidate)) {
            gt_csv = candidate;
            break;
        }
    }
    if (!gt_csv.empty()) {
        loadGt(gt_csv.string());
    }

    // ── Build synthetic LiDAR timeline at 10 Hz ─────────────────────────
    if (!imu_rows_.empty()) {
        int64_t first_ts = imu_rows_.front().ts_ns;
        int64_t last_ts  = imu_rows_.back().ts_ns;
        int64_t step_ns  = 100'000'000;  // 100 ms = 10 Hz

        for (int64_t t = first_ts; t <= last_ts; t += step_ns) {
            lidar_timestamps_ns_.push_back(t);
        }
    }

    buildTimeline();

    std::cerr << "[euroc] opened: " << path
              << " | IMU: " << imu_rows_.size()
              << " | GT: " << gt_rows_.size()
              << " | synth LiDAR: " << lidar_timestamps_ns_.size()
              << " frames\n";

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  info()
// ─────────────────────────────────────────────────────────────────────────────

DatasetInfo EurocAdapter::info() const {
    DatasetInfo di;
    di.name   = fs::path(seq_path_).filename().string();
    di.format = "euroc";
    di.has_lidar = !lidar_timestamps_ns_.empty();
    di.has_imu   = !imu_rows_.empty();
    di.has_ground_truth = !gt_rows_.empty();

    if (!imu_rows_.empty()) {
        di.duration_s = (imu_rows_.back().ts_ns - imu_rows_.front().ts_ns) / 1.0e9;
    }
    if (!gt_rows_.empty()) {
        double dist = 0.0;
        for (size_t i = 1; i < gt_rows_.size(); ++i) {
            double dx = gt_rows_[i].px - gt_rows_[i-1].px;
            double dy = gt_rows_[i].py - gt_rows_[i-1].py;
            double dz = gt_rows_[i].pz - gt_rows_[i-1].pz;
            dist += std::sqrt(dx*dx + dy*dy + dz*dz);
        }
        di.distance_m = dist;
    }

    return di;
}

// ─────────────────────────────────────────────────────────────────────────────
//  next()
// ─────────────────────────────────────────────────────────────────────────────

std::optional<StreamEvent> EurocAdapter::next() {
    while (timeline_cursor_ < timeline_.size()) {
        const auto& te = timeline_[timeline_cursor_++];

        if (te.type == TimelineEvent::LIDAR) {
            if (max_frames_ > 0 && frames_emitted_ >= max_frames_) {
                return std::nullopt;
            }
            auto cloud = synthCloud(te.index);
            if (!cloud) continue;

            StreamEvent ev;
            ev.timestamp_ns = cloud->timestamp_ns;
            ev.payload      = std::move(cloud);
            ++frames_emitted_;
            return ev;
        }

        if (te.type == TimelineEvent::IMU && te.index < imu_rows_.size()) {
            const auto& row = imu_rows_[te.index];
            thunderbird::odom::ImuSample sample;
            sample.timestamp_ns = row.ts_ns;
            sample.gyro  = {row.wx, row.wy, row.wz};
            sample.accel = {row.ax, row.ay, row.az};
            sample.temperature = 25.0;

            StreamEvent ev;
            ev.timestamp_ns = row.ts_ns;
            ev.payload      = sample;
            return ev;
        }
    }
    return std::nullopt;
}

// ─────────────────────────────────────────────────────────────────────────────
//  loadGroundTruth()
// ─────────────────────────────────────────────────────────────────────────────

std::vector<GtPose> EurocAdapter::loadGroundTruth() {
    std::vector<GtPose> poses;
    poses.reserve(gt_rows_.size());
    for (const auto& r : gt_rows_) {
        GtPose gp;
        gp.timestamp_ns = r.ts_ns;
        gp.position     = {r.px, r.py, r.pz};
        gp.quaternion   = {r.qw, r.qx, r.qy, r.qz};
        poses.push_back(gp);
    }
    std::cerr << "[euroc] loaded " << poses.size() << " ground truth poses\n";
    return poses;
}

// ─────────────────────────────────────────────────────────────────────────────
//  rewind()
// ─────────────────────────────────────────────────────────────────────────────

void EurocAdapter::rewind() {
    timeline_cursor_ = 0;
    frames_emitted_  = 0;
    imu_cursor_      = 0;
    lidar_cursor_    = 0;
    timer_.reset();
}

// ═════════════════════════════════════════════════════════════════════════════
//  Private helpers
// ═════════════════════════════════════════════════════════════════════════════

bool EurocAdapter::loadImu(const std::string& csv_path) {
    std::ifstream f(csv_path);
    if (!f.is_open()) return false;

    imu_rows_.clear();
    std::string line;

    // Skip header line.
    std::getline(f, line);

    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;

        // Format: timestamp[ns],wx,wy,wz,ax,ay,az
        // Replace commas with spaces for easier parsing.
        for (char& c : line) if (c == ',') c = ' ';

        std::istringstream iss(line);
        ImuRow row;
        int64_t ts;
        if (!(iss >> ts >> row.wx >> row.wy >> row.wz
                      >> row.ax >> row.ay >> row.az)) continue;
        row.ts_ns = ts;
        imu_rows_.push_back(row);
    }

    // Sort by timestamp (should already be sorted).
    std::sort(imu_rows_.begin(), imu_rows_.end(),
              [](const ImuRow& a, const ImuRow& b) { return a.ts_ns < b.ts_ns; });

    return !imu_rows_.empty();
}

bool EurocAdapter::loadGt(const std::string& csv_path) {
    std::ifstream f(csv_path);
    if (!f.is_open()) return false;

    gt_rows_.clear();
    std::string line;

    // Skip header.
    std::getline(f, line);

    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;

        for (char& c : line) if (c == ',') c = ' ';

        std::istringstream iss(line);
        GtRow row;
        int64_t ts;
        // Format: timestamp[ns],px,py,pz,qw,qx,qy,qz,vx,vy,vz,...
        // (additional columns are velocity/bias — ignored)
        if (!(iss >> ts >> row.px >> row.py >> row.pz
                      >> row.qw >> row.qx >> row.qy >> row.qz)) continue;
        row.ts_ns = ts;
        gt_rows_.push_back(row);
    }

    std::sort(gt_rows_.begin(), gt_rows_.end(),
              [](const GtRow& a, const GtRow& b) { return a.ts_ns < b.ts_ns; });

    return !gt_rows_.empty();
}

void EurocAdapter::buildTimeline() {
    timeline_.clear();
    timeline_.reserve(imu_rows_.size() + lidar_timestamps_ns_.size());

    for (size_t i = 0; i < imu_rows_.size(); ++i) {
        timeline_.push_back({imu_rows_[i].ts_ns, TimelineEvent::IMU, i});
    }
    for (size_t i = 0; i < lidar_timestamps_ns_.size(); ++i) {
        timeline_.push_back({lidar_timestamps_ns_[i], TimelineEvent::LIDAR, i});
    }

    // Sort: timestamp primary, IMU before LIDAR at equal timestamp
    // (engine needs IMU preintegrated before scan registration).
    std::sort(timeline_.begin(), timeline_.end(),
              [](const TimelineEvent& a, const TimelineEvent& b) {
                  if (a.ts_ns != b.ts_ns) return a.ts_ns < b.ts_ns;
                  return a.type < b.type;  // IMU=0 < LIDAR=1
              });
}

/// Generate a synthetic point cloud at the given lidar timeline index.
/// Creates a simple structured environment (walls + ground plane) around
/// the GT position at that timestamp.
std::shared_ptr<const PointCloudFrame> EurocAdapter::synthCloud(size_t lidar_idx) {
    if (lidar_idx >= lidar_timestamps_ns_.size()) return nullptr;

    int64_t ts_ns = lidar_timestamps_ns_[lidar_idx];

    // Find nearest GT pose.
    double px = 0, py = 0, pz = 0;
    double qw = 1, qx = 0, qy = 0, qz = 0;
    if (!gt_rows_.empty()) {
        auto it = std::lower_bound(gt_rows_.begin(), gt_rows_.end(), ts_ns,
            [](const GtRow& g, int64_t t) { return g.ts_ns < t; });
        size_t idx = (it == gt_rows_.end())
            ? gt_rows_.size() - 1
            : static_cast<size_t>(it - gt_rows_.begin());
        if (idx > 0 && it != gt_rows_.begin()) {
            // Check prev vs current.
            auto prev = std::prev(it);
            if (std::abs(prev->ts_ns - ts_ns) < std::abs(it->ts_ns - ts_ns)) {
                idx = static_cast<size_t>(prev - gt_rows_.begin());
            }
        }
        px = gt_rows_[idx].px; py = gt_rows_[idx].py; pz = gt_rows_[idx].pz;
        qw = gt_rows_[idx].qw; qx = gt_rows_[idx].qx;
        qy = gt_rows_[idx].qy; qz = gt_rows_[idx].qz;
    }

    // Generate structured point cloud in world frame, then transform
    // to body frame using inverse of GT pose.
    auto cloud = std::make_shared<PointCloudFrame>();
    cloud->timestamp_ns = ts_ns;
    cloud->sequence     = static_cast<uint32_t>(lidar_idx);
    cloud->is_deskewed  = true;  // synthetic, already aligned

    // Inverse rotation: q_inv = conjugate(q) / |q|² (unit quat → just conj)
    double qw_inv = qw, qx_inv = -qx, qy_inv = -qy, qz_inv = -qz;

    // Generate points on ground plane, walls, and ceiling.
    const int n_ground = 400;
    const int n_wall   = 200;
    cloud->points.reserve(n_ground + n_wall * 4);

    // Ground plane: z = 0, grid around current position.
    for (int i = 0; i < n_ground; ++i) {
        double angle  = 2.0 * M_PI * i / n_ground;
        double radius = 2.0 + 8.0 * static_cast<double>(i % 20) / 20.0;

        // World-frame point.
        double wx = px + radius * std::cos(angle);
        double wy = py + radius * std::sin(angle);
        double wz = 0.0;  // ground

        // To body frame: p_body = R_inv * (p_world - t)
        double dx = wx - px, dy = wy - py, dz = wz - pz;
        auto body = rotateByQuat(qw_inv, qx_inv, qy_inv, qz_inv, dx, dy, dz);

        thunderbird::odom::PointXYZIT pt;
        pt.x = static_cast<float>(body[0]);
        pt.y = static_cast<float>(body[1]);
        pt.z = static_cast<float>(body[2]);
        pt.intensity = 50.0f;
        pt.dt_ns = 0;
        cloud->points.push_back(pt);
    }

    // Walls at ±5m in X and Y (world frame).
    for (int wall = 0; wall < 4; ++wall) {
        for (int i = 0; i < n_wall; ++i) {
            double frac = static_cast<double>(i) / n_wall;
            double wx, wy, wz;
            wz = pz - 1.0 + 3.0 * frac;  // -1 to +2 m relative

            switch (wall) {
                case 0: wx = px + 5.0; wy = py - 5.0 + 10.0 * frac; break;
                case 1: wx = px - 5.0; wy = py - 5.0 + 10.0 * frac; break;
                case 2: wx = px - 5.0 + 10.0 * frac; wy = py + 5.0; break;
                default: wx = px - 5.0 + 10.0 * frac; wy = py - 5.0; break;
            }

            double dx = wx - px, dy = wy - py, dz = wz - pz;
            auto body = rotateByQuat(qw_inv, qx_inv, qy_inv, qz_inv, dx, dy, dz);

            thunderbird::odom::PointXYZIT pt;
            pt.x = static_cast<float>(body[0]);
            pt.y = static_cast<float>(body[1]);
            pt.z = static_cast<float>(body[2]);
            pt.intensity = 80.0f;
            pt.dt_ns = 0;
            cloud->points.push_back(pt);
        }
    }

    return cloud;
}

std::array<double,3> EurocAdapter::rotateByQuat(
    double qw, double qx, double qy, double qz,
    double vx, double vy, double vz)
{
    // q ⊗ [0,v] ⊗ q*  (expanded)
    double ux = qx, uy = qy, uz = qz;
    double s  = qw;

    // t = 2 * cross(u, v)
    double tx = 2.0 * (uy * vz - uz * vy);
    double ty = 2.0 * (uz * vx - ux * vz);
    double tz = 2.0 * (ux * vy - uy * vx);

    return {
        vx + s * tx + (uy * tz - uz * ty),
        vy + s * ty + (uz * tx - ux * tz),
        vz + s * tz + (ux * ty - uy * tx),
    };
}

// ═════════════════════════════════════════════════════════════════════════════
//  Factory hook
// ═════════════════════════════════════════════════════════════════════════════

std::unique_ptr<DatasetAdapter> make_euroc_adapter() {
    return std::make_unique<EurocAdapter>();
}

} // namespace eval
