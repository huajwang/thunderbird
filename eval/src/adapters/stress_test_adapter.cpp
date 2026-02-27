// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Evaluation Framework: Stress-Test Adapter Impl
// ─────────────────────────────────────────────────────────────────────────────

#include "eval/stress_test_adapter.h"
#include "eval/imu_interpolator.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace eval {

// ═════════════════════════════════════════════════════════════════════════════
//  Named presets
// ═════════════════════════════════════════════════════════════════════════════

StressConfig StressConfig::aggressiveDrone() {
    StressConfig c;
    c.duration_s            = 60.0;
    c.lidar_rate_hz         = 10.0;
    c.imu_rate_hz           = 400.0;
    c.max_linear_speed_mps  = 15.0;   // 54 km/h
    c.max_angular_rate_rps  = 6.0;    // ~344 deg/s
    c.jerk_factor           = 3.0;    // violent direction changes
    c.direction_change_freq = 2.0;    // 2 Hz direction shocks
    c.min_points            = 2000;
    c.max_points            = 40000;
    c.feature_density       = 0.6;    // partially sparse
    c.max_range_m           = 60.0;
    c.accel_noise_sigma     = 0.05;
    c.gyro_noise_sigma      = 0.005;
    c.lidar_noise_sigma     = 0.02;
    c.seed                  = 42;
    return c;
}

StressConfig StressConfig::fastCar() {
    StressConfig c;
    c.duration_s            = 120.0;
    c.lidar_rate_hz         = 10.0;
    c.imu_rate_hz           = 200.0;
    c.max_linear_speed_mps  = 33.3;   // 120 km/h
    c.max_angular_rate_rps  = 1.5;    // ~86 deg/s
    c.jerk_factor           = 2.0;
    c.direction_change_freq = 0.3;
    c.min_points            = 10000;
    c.max_points            = 80000;
    c.feature_density       = 1.0;
    c.max_range_m           = 100.0;
    c.accel_noise_sigma     = 0.03;
    c.gyro_noise_sigma      = 0.003;
    c.lidar_noise_sigma     = 0.015;
    c.seed                  = 7777;
    return c;
}

StressConfig StressConfig::degenerateCorridor() {
    StressConfig c;
    c.duration_s            = 45.0;
    c.lidar_rate_hz         = 10.0;
    c.imu_rate_hz           = 200.0;
    c.max_linear_speed_mps  = 2.0;
    c.max_angular_rate_rps  = 0.3;
    c.jerk_factor           = 0.5;
    c.direction_change_freq = 0.1;
    c.min_points            = 200;      // very sparse
    c.max_points            = 1500;
    c.feature_density       = 0.1;      // degenerate geometry
    c.max_range_m           = 30.0;
    c.accel_noise_sigma     = 0.01;
    c.gyro_noise_sigma      = 0.001;
    c.lidar_noise_sigma     = 0.005;
    c.seed                  = 31415;
    return c;
}

StressConfig StressConfig::spinningTop() {
    StressConfig c;
    c.duration_s            = 20.0;
    c.lidar_rate_hz         = 10.0;
    c.imu_rate_hz           = 500.0;    // high rate for extreme rotation
    c.max_linear_speed_mps  = 0.1;      // nearly stationary
    c.max_angular_rate_rps  = 10.0;     // ~573 deg/s
    c.jerk_factor           = 4.0;
    c.direction_change_freq = 3.0;
    c.min_points            = 5000;
    c.max_points            = 30000;
    c.feature_density       = 0.8;
    c.max_range_m           = 50.0;
    c.accel_noise_sigma     = 0.1;      // high noise
    c.gyro_noise_sigma      = 0.02;
    c.lidar_noise_sigma     = 0.03;
    c.seed                  = 99999;
    return c;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Construction
// ═════════════════════════════════════════════════════════════════════════════

StressTestAdapter::StressTestAdapter(StressConfig cfg)
    : cfg_(cfg), rng_(cfg.seed) {}

// ═════════════════════════════════════════════════════════════════════════════
//  open()
// ═════════════════════════════════════════════════════════════════════════════

bool StressTestAdapter::open(const std::string& path, size_t max_frames) {
    // path is used as the preset name (or "custom").
    preset_name_ = path.empty() ? "custom" : path;
    max_frames_  = max_frames;
    rng_.seed(cfg_.seed);

    generateTrajectory();

    std::cerr << "[stress] opened preset '" << preset_name_ << "': "
              << gt_poses_.size() << " LiDAR frames, "
              << imu_samples_.size() << " IMU samples, "
              << cfg_.duration_s << " s\n";

    rewind();
    return true;
}

// ═════════════════════════════════════════════════════════════════════════════
//  info()
// ═════════════════════════════════════════════════════════════════════════════

DatasetInfo StressTestAdapter::info() const {
    DatasetInfo di;
    di.name     = "stress_" + preset_name_;
    di.format   = "stress";
    di.has_lidar = true;
    di.has_imu   = true;
    di.has_ground_truth = true;
    di.duration_s = cfg_.duration_s;
    return di;
}

// ═════════════════════════════════════════════════════════════════════════════
//  next()  — interleave IMU + LiDAR in timestamp order
// ═════════════════════════════════════════════════════════════════════════════

std::optional<StreamEvent> StressTestAdapter::next() {
    const size_t eff_max = (max_frames_ > 0)
        ? std::min(max_frames_, gt_poses_.size())
        : gt_poses_.size();

    const bool have_imu   = imu_cursor_ < imu_samples_.size();
    const bool have_lidar = lidar_cursor_ < eff_max;

    if (!have_imu && !have_lidar) return std::nullopt;

    // Determine the LiDAR timestamp for the current frame.
    int64_t next_lidar_ts = have_lidar
        ? gt_poses_[lidar_cursor_].timestamp_ns
        : INT64_MAX;

    // Emit all IMU samples before the next LiDAR frame.
    if (have_imu && imu_samples_[imu_cursor_].timestamp_ns < next_lidar_ts) {
        StreamEvent ev;
        ev.timestamp_ns = imu_samples_[imu_cursor_].timestamp_ns;
        ev.payload      = imu_samples_[imu_cursor_];
        ++imu_cursor_;
        return ev;
    }

    // Emit LiDAR frame.
    if (have_lidar) {
        auto cloud = generateCloud(lidar_cursor_,
                                    gt_poses_[lidar_cursor_].timestamp_ns,
                                    gt_poses_[lidar_cursor_]);
        StreamEvent ev;
        ev.timestamp_ns = cloud->timestamp_ns;
        ev.payload      = std::move(cloud);
        ++lidar_cursor_;
        ++lidar_emitted_;
        return ev;
    }

    // Only IMU remaining.
    if (have_imu) {
        StreamEvent ev;
        ev.timestamp_ns = imu_samples_[imu_cursor_].timestamp_ns;
        ev.payload      = imu_samples_[imu_cursor_];
        ++imu_cursor_;
        return ev;
    }

    return std::nullopt;
}

// ═════════════════════════════════════════════════════════════════════════════
//  loadGroundTruth()
// ═════════════════════════════════════════════════════════════════════════════

std::vector<GtPose> StressTestAdapter::loadGroundTruth() {
    return gt_poses_;
}

// ═════════════════════════════════════════════════════════════════════════════
//  rewind()
// ═════════════════════════════════════════════════════════════════════════════

void StressTestAdapter::rewind() {
    lidar_cursor_  = 0;
    imu_cursor_    = 0;
    lidar_emitted_ = 0;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Trajectory generation — aggressive kinematic profile
// ═════════════════════════════════════════════════════════════════════════════

void StressTestAdapter::generateTrajectory() {
    gt_poses_.clear();
    imu_samples_.clear();

    const double dt_lidar = 1.0 / cfg_.lidar_rate_hz;
    const size_t num_lidar = static_cast<size_t>(cfg_.duration_s * cfg_.lidar_rate_hz);

    // ── Build GT poses using piecewise aggressive motion ────────────────
    //
    // State: position (x,y,z), velocity (vx,vy,vz), orientation as Euler
    // angles (roll, pitch, yaw) — converted to quaternion for output.

    std::normal_distribution<double> jerk_dist(0.0, cfg_.jerk_factor);
    std::uniform_real_distribution<double> dir_dist(-1.0, 1.0);
    std::uniform_real_distribution<double> rate_dist(-cfg_.max_angular_rate_rps,
                                                      cfg_.max_angular_rate_rps);

    double x = 0, y = 0, z = 0;
    double vx = 0, vy = 0, vz = 0;
    double roll = 0, pitch = 0, yaw = 0;
    double omega_roll = 0, omega_pitch = 0, omega_yaw = 0;

    // Time since last direction change.
    double since_change = 0.0;
    const double change_interval = 1.0 / std::max(cfg_.direction_change_freq, 0.01);

    for (size_t i = 0; i < num_lidar; ++i) {
        const double t = static_cast<double>(i) * dt_lidar;

        // ── Maybe inject a direction change ─────────────────────────────
        since_change += dt_lidar;
        if (since_change >= change_interval) {
            since_change = 0.0;

            // Random velocity impulse.
            double speed_impulse = cfg_.max_linear_speed_mps * jerk_dist(rng_) * 0.5;
            vx += speed_impulse * dir_dist(rng_);
            vy += speed_impulse * dir_dist(rng_);
            vz += speed_impulse * dir_dist(rng_) * 0.3;  // less vertical

            // Random angular velocity change.
            omega_roll  = rate_dist(rng_) * cfg_.jerk_factor;
            omega_pitch = rate_dist(rng_) * cfg_.jerk_factor * 0.5;
            omega_yaw   = rate_dist(rng_) * cfg_.jerk_factor;
        }

        // ── Clamp speeds ────────────────────────────────────────────────
        double speed = std::sqrt(vx*vx + vy*vy + vz*vz);
        if (speed > cfg_.max_linear_speed_mps) {
            double scale = cfg_.max_linear_speed_mps / speed;
            vx *= scale; vy *= scale; vz *= scale;
        }

        omega_roll  = std::clamp(omega_roll,  -cfg_.max_angular_rate_rps, cfg_.max_angular_rate_rps);
        omega_pitch = std::clamp(omega_pitch, -cfg_.max_angular_rate_rps, cfg_.max_angular_rate_rps);
        omega_yaw   = std::clamp(omega_yaw,   -cfg_.max_angular_rate_rps, cfg_.max_angular_rate_rps);

        // ── Integrate ───────────────────────────────────────────────────
        x += vx * dt_lidar;
        y += vy * dt_lidar;
        z += vz * dt_lidar;

        roll  += omega_roll  * dt_lidar;
        pitch += omega_pitch * dt_lidar;
        yaw   += omega_yaw   * dt_lidar;

        // ── Euler → quaternion (ZYX convention) ─────────────────────────
        double cr = std::cos(roll/2),  sr = std::sin(roll/2);
        double cp = std::cos(pitch/2), sp = std::sin(pitch/2);
        double cy = std::cos(yaw/2),   sy = std::sin(yaw/2);

        GtPose gp;
        gp.timestamp_ns = static_cast<int64_t>(std::llround(t * 1.0e9));
        gp.position     = {x, y, z};
        gp.quaternion   = {
            cr*cp*cy + sr*sp*sy,   // w
            sr*cp*cy - cr*sp*sy,   // x
            cr*sp*cy + sr*cp*sy,   // y
            cr*cp*sy - sr*sp*cy    // z
        };

        // Normalise.
        double qn = std::sqrt(gp.quaternion[0]*gp.quaternion[0] +
                              gp.quaternion[1]*gp.quaternion[1] +
                              gp.quaternion[2]*gp.quaternion[2] +
                              gp.quaternion[3]*gp.quaternion[3]);
        if (qn > 0) for (auto& q : gp.quaternion) q /= qn;

        gt_poses_.push_back(gp);

        // ── Apply drag (smooth deceleration) ────────────────────────────
        const double drag = 0.98;
        vx *= drag; vy *= drag; vz *= drag;
        omega_roll  *= drag;
        omega_pitch *= drag;
        omega_yaw   *= drag;
    }

    // ── Generate synthetic IMU from GT poses via ImuInterpolator ────────
    if (gt_poses_.size() >= 2) {
        ImuInterpolatorConfig imu_cfg;
        imu_cfg.imu_rate_hz       = cfg_.imu_rate_hz;
        imu_cfg.accel_noise_sigma = cfg_.accel_noise_sigma;
        imu_cfg.gyro_noise_sigma  = cfg_.gyro_noise_sigma;
        imu_cfg.add_noise         = true;
        imu_cfg.noise_seed        = cfg_.seed + 1;

        ImuInterpolator interp(imu_cfg);
        interp.prepare(gt_poses_);
        imu_samples_ = interp.generateAll();
    }

    std::cerr << "[stress] trajectory: " << gt_poses_.size() << " poses, "
              << imu_samples_.size() << " IMU samples\n";
}

// ═════════════════════════════════════════════════════════════════════════════
//  Point cloud generation — synthetic geometry with configurable density
// ═════════════════════════════════════════════════════════════════════════════

std::shared_ptr<const PointCloudFrame> StressTestAdapter::generateCloud(
    size_t frame_idx, int64_t timestamp_ns, const GtPose& pose)
{
    // Number of points: modulated by feature_density and random variation.
    std::uniform_int_distribution<size_t> pts_dist(cfg_.min_points, cfg_.max_points);
    size_t num_pts = static_cast<size_t>(
        static_cast<double>(pts_dist(rng_)) * cfg_.feature_density);
    num_pts = std::max(num_pts, cfg_.min_points);

    std::normal_distribution<double> range_dist(cfg_.max_range_m * 0.4,
                                                 cfg_.max_range_m * 0.3);
    std::uniform_real_distribution<double> angle_dist(-M_PI, M_PI);
    std::uniform_real_distribution<double> elev_dist(-0.4, 0.2);  // slight downward bias
    std::normal_distribution<double> noise_dist(0.0, cfg_.lidar_noise_sigma);

    auto cloud = std::make_shared<PointCloudFrame>();
    cloud->timestamp_ns = timestamp_ns;
    cloud->sequence     = static_cast<uint32_t>(frame_idx);
    cloud->is_deskewed  = false;
    cloud->points.resize(num_pts);

    const double scan_period_ns = 1.0e9 / cfg_.lidar_rate_hz;  // ~100 ms

    for (size_t i = 0; i < num_pts; ++i) {
        double range = std::abs(range_dist(rng_));
        range = std::min(range, cfg_.max_range_m);

        double azimuth   = angle_dist(rng_);
        double elevation = elev_dist(rng_);

        auto& pt = cloud->points[i];
        pt.x = static_cast<float>(range * std::cos(elevation) * std::cos(azimuth) + noise_dist(rng_));
        pt.y = static_cast<float>(range * std::cos(elevation) * std::sin(azimuth) + noise_dist(rng_));
        pt.z = static_cast<float>(range * std::sin(elevation) + noise_dist(rng_));
        pt.intensity = static_cast<float>(std::abs(range_dist(rng_)) / cfg_.max_range_m * 255.0);
        pt.dt_ns = static_cast<int32_t>(
            scan_period_ns * static_cast<double>(i) / static_cast<double>(num_pts));
    }

    return cloud;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Factory
// ═════════════════════════════════════════════════════════════════════════════

std::unique_ptr<DatasetAdapter> make_stress_adapter(StressConfig cfg) {
    return std::make_unique<StressTestAdapter>(cfg);
}

} // namespace eval
