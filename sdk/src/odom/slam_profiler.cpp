// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Profiler implementation
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/odom/slam_profiler.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <numeric>
#include <sstream>

#ifdef __linux__
#include <fstream>
#include <unistd.h>
#include <sys/resource.h>
#endif

namespace thunderbird::odom {

// ═════════════════════════════════════════════════════════════════════════════
//  Platform helpers
// ═════════════════════════════════════════════════════════════════════════════

namespace {

#ifdef __linux__
/// Read /proc/self/status for memory info.
MemoryMetrics read_proc_memory() noexcept {
    MemoryMetrics m{};
    std::ifstream status("/proc/self/status");
    if (!status.is_open()) return m;

    std::string line;
    while (std::getline(status, line)) {
        if (line.compare(0, 6, "VmRSS:") == 0) {
            // Format: "VmRSS:    12345 kB"
            const char* p = line.c_str() + 6;
            while (*p == ' ' || *p == '\t') ++p;
            m.rss_bytes = static_cast<size_t>(std::atoll(p)) * 1024;
        } else if (line.compare(0, 7, "VmSize:") == 0) {
            const char* p = line.c_str() + 7;
            while (*p == ' ' || *p == '\t') ++p;
            m.vm_bytes = static_cast<size_t>(std::atoll(p)) * 1024;
        }
    }
    return m;
}

/// Read CPU times from getrusage.
CpuMetrics read_cpu_usage(double wall_time_s) noexcept {
    CpuMetrics c{};
    struct rusage ru{};
    if (::getrusage(RUSAGE_SELF, &ru) == 0) {
        c.user_time_s   = static_cast<double>(ru.ru_utime.tv_sec)
                        + static_cast<double>(ru.ru_utime.tv_usec) / 1.0e6;
        c.system_time_s  = static_cast<double>(ru.ru_stime.tv_sec)
                        + static_cast<double>(ru.ru_stime.tv_usec) / 1.0e6;
        c.wall_time_s    = wall_time_s;
        if (wall_time_s > 0.0) {
            c.process_cpu_pct = (c.user_time_s + c.system_time_s)
                              / wall_time_s * 100.0;
        }
    }
    return c;
}
#else
/// Stub for non-Linux platforms.
[[maybe_unused]]
MemoryMetrics read_proc_memory() noexcept { return {}; }
[[maybe_unused]]
CpuMetrics read_cpu_usage(double wall_time_s) noexcept {
    CpuMetrics c{};
    c.wall_time_s = wall_time_s;
    return c;
}
#endif

/// Quaternion to rotation angle (radians).
double quaternion_angle_diff(const std::array<double,4>& q1,
                             const std::array<double,4>& q2) noexcept {
    // q_diff = q1^{-1} * q2
    // For unit quaternions: q1^{-1} = conjugate(q1)
    // angle = 2 * acos(|q_diff.w|)
    const double dot = q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3];
    const double clamped = std::min(1.0, std::abs(dot));
    return 2.0 * std::acos(clamped);
}

/// Euclidean distance between two 3D points.
double position_distance(const std::array<double,3>& a,
                          const std::array<double,3>& b) noexcept {
    const double dx = a[0] - b[0];
    const double dy = a[1] - b[1];
    const double dz = a[2] - b[2];
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

} // anonymous namespace

// ═════════════════════════════════════════════════════════════════════════════
//  SlamProfiler — snapshot
// ═════════════════════════════════════════════════════════════════════════════

SlamProfileSnapshot SlamProfiler::snapshot() const {
    SlamProfileSnapshot snap;
    snap.timestamp_ns = profiler_now_ns();
    snap.uptime_s = static_cast<double>(snap.timestamp_ns - start_time_ns_) / 1.0e9;

    // ── Probes ──────────────────────────────────────────────────────────
    const size_t n = probe_count_.load(std::memory_order_acquire);
    snap.probes.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        const auto& p = probes_[i];
        ProbeSnapshot ps;
        ps.name     = p.name;
        ps.count    = p.count;
        ps.mean_us  = p.mean_us();
        ps.min_us   = (p.min_ns == INT64_MAX) ? 0.0
                     : static_cast<double>(p.min_ns) / 1000.0;
        ps.max_us   = static_cast<double>(p.max_ns) / 1000.0;
        ps.p50_us   = p.percentile_us(50.0);
        ps.p95_us   = p.percentile_us(95.0);
        ps.p99_us   = p.percentile_us(99.0);
        ps.total_ms = static_cast<double>(p.sum_ns) / 1.0e6;
        ps.histogram = p.histogram;
        snap.probes.push_back(std::move(ps));
    }

    // ── System metrics ──────────────────────────────────────────────────
    snap.memory = memory_;
    snap.cpu    = cpu_;
    snap.drift  = drift_;

    return snap;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Drift evaluation
// ═════════════════════════════════════════════════════════════════════════════

void SlamProfiler::feed_ground_truth(const GroundTruthPose& gt) noexcept {
    gt_poses_.push_back({gt.timestamp_ns, gt.position, gt.quaternion});
}

void SlamProfiler::feed_estimated_pose(
    int64_t timestamp_ns,
    const std::array<double,3>& position,
    const std::array<double,4>& quaternion) noexcept {
    est_poses_.push_back({timestamp_ns, position, quaternion});
}

void SlamProfiler::compute_drift() noexcept {
    if (gt_poses_.empty() || est_poses_.empty()) return;

    // ── Sort both by timestamp ──────────────────────────────────────────
    auto ts_cmp = [](const TimestampedPosition& a,
                     const TimestampedPosition& b) {
        return a.timestamp_ns < b.timestamp_ns;
    };
    std::sort(gt_poses_.begin(), gt_poses_.end(), ts_cmp);
    std::sort(est_poses_.begin(), est_poses_.end(), ts_cmp);

    // ── Match estimated to nearest GT (within 10 ms) ────────────────────
    DriftMetrics dm{};
    double sum_pos_err_sq   = 0.0;
    double sum_rot_err      = 0.0;
    double max_pos_err      = 0.0;
    size_t matched          = 0;
    double total_distance   = 0.0;

    // Compute cumulative distance from estimated trajectory.
    for (size_t i = 1; i < est_poses_.size(); ++i) {
        total_distance += position_distance(
            est_poses_[i-1].position, est_poses_[i].position);
    }

    // Nearest-neighbor matching with sliding window.
    size_t gt_idx = 0;
    constexpr int64_t kMatchWindow = 10'000'000; // 10 ms

    for (const auto& est : est_poses_) {
        // Advance GT index to closest.
        while (gt_idx + 1 < gt_poses_.size() &&
               std::abs(gt_poses_[gt_idx + 1].timestamp_ns - est.timestamp_ns) <
               std::abs(gt_poses_[gt_idx].timestamp_ns - est.timestamp_ns)) {
            ++gt_idx;
        }

        if (gt_idx >= gt_poses_.size()) break;
        if (std::abs(gt_poses_[gt_idx].timestamp_ns - est.timestamp_ns) > kMatchWindow)
            continue;

        const double pos_err = position_distance(
            est.position, gt_poses_[gt_idx].position);
        const double rot_err = quaternion_angle_diff(
            est.quaternion, gt_poses_[gt_idx].quaternion);

        sum_pos_err_sq += pos_err * pos_err;
        sum_rot_err    += rot_err;
        if (pos_err > max_pos_err) max_pos_err = pos_err;
        ++matched;
    }

    if (matched > 0) {
        dm.distance_traveled_m      = total_distance;
        dm.absolute_position_error_m = std::sqrt(sum_pos_err_sq / static_cast<double>(matched));
        dm.max_position_error_m     = max_pos_err;
        dm.mean_rotation_error_deg  = (sum_rot_err / static_cast<double>(matched))
                                     * (180.0 / 3.14159265358979323846);
        dm.num_ground_truth_points  = matched;
        if (total_distance > 0.0) {
            dm.relative_drift_pct = (dm.absolute_position_error_m / total_distance) * 100.0;
        }
    }

    drift_ = dm;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Report generation — JSON
// ═════════════════════════════════════════════════════════════════════════════

std::string SlamProfiler::report_json(const SlamProfileSnapshot& snap) {
    std::ostringstream os;
    os << "{\n";
    os << "  \"timestamp_ns\": " << snap.timestamp_ns << ",\n";
    os << "  \"uptime_s\": " << snap.uptime_s << ",\n";

    // ── Probes ──────────────────────────────────────────────────────────
    os << "  \"probes\": [\n";
    for (size_t i = 0; i < snap.probes.size(); ++i) {
        const auto& p = snap.probes[i];
        os << "    {\n";
        os << "      \"name\": \"" << p.name << "\",\n";
        os << "      \"count\": " << p.count << ",\n";
        os << "      \"mean_us\": " << p.mean_us << ",\n";
        os << "      \"min_us\": " << p.min_us << ",\n";
        os << "      \"max_us\": " << p.max_us << ",\n";
        os << "      \"p50_us\": " << p.p50_us << ",\n";
        os << "      \"p95_us\": " << p.p95_us << ",\n";
        os << "      \"p99_us\": " << p.p99_us << ",\n";
        os << "      \"total_ms\": " << p.total_ms << ",\n";
        os << "      \"histogram\": [";
        for (size_t b = 0; b < kHistogramBuckets; ++b) {
            if (b > 0) os << ", ";
            os << p.histogram[b];
        }
        os << "]\n";
        os << "    }";
        if (i + 1 < snap.probes.size()) os << ",";
        os << "\n";
    }
    os << "  ],\n";

    // ── Memory ──────────────────────────────────────────────────────────
    os << "  \"memory\": {\n";
    os << "    \"rss_bytes\": " << snap.memory.rss_bytes << ",\n";
    os << "    \"vm_bytes\": " << snap.memory.vm_bytes << ",\n";
    os << "    \"map_points\": " << snap.memory.map_points << ",\n";
    os << "    \"map_bytes_estimate\": " << snap.memory.map_bytes_estimate << ",\n";
    os << "    \"ring_buffer_bytes\": " << snap.memory.ring_buffer_bytes << "\n";
    os << "  },\n";

    // ── CPU ─────────────────────────────────────────────────────────────
    os << "  \"cpu\": {\n";
    os << "    \"process_cpu_pct\": " << snap.cpu.process_cpu_pct << ",\n";
    os << "    \"worker_thread_cpu_pct\": " << snap.cpu.worker_thread_cpu_pct << ",\n";
    os << "    \"wall_time_s\": " << snap.cpu.wall_time_s << ",\n";
    os << "    \"user_time_s\": " << snap.cpu.user_time_s << ",\n";
    os << "    \"system_time_s\": " << snap.cpu.system_time_s << "\n";
    os << "  },\n";

    // ── Drift ───────────────────────────────────────────────────────────
    os << "  \"drift\": {\n";
    os << "    \"distance_traveled_m\": " << snap.drift.distance_traveled_m << ",\n";
    os << "    \"absolute_position_error_m\": " << snap.drift.absolute_position_error_m << ",\n";
    os << "    \"relative_drift_pct\": " << snap.drift.relative_drift_pct << ",\n";
    os << "    \"max_position_error_m\": " << snap.drift.max_position_error_m << ",\n";
    os << "    \"mean_rotation_error_deg\": " << snap.drift.mean_rotation_error_deg << ",\n";
    os << "    \"num_ground_truth_points\": " << snap.drift.num_ground_truth_points << "\n";
    os << "  }\n";

    os << "}\n";
    return os.str();
}

// ═════════════════════════════════════════════════════════════════════════════
//  Report generation — CSV
// ═════════════════════════════════════════════════════════════════════════════

std::string SlamProfiler::report_csv(const SlamProfileSnapshot& snap) {
    std::ostringstream os;
    os << "probe,count,mean_us,min_us,max_us,p50_us,p95_us,p99_us,total_ms\n";
    for (const auto& p : snap.probes) {
        os << p.name << ","
           << p.count << ","
           << p.mean_us << ","
           << p.min_us << ","
           << p.max_us << ","
           << p.p50_us << ","
           << p.p95_us << ","
           << p.p99_us << ","
           << p.total_ms << "\n";
    }
    return os.str();
}

// ═════════════════════════════════════════════════════════════════════════════
//  Report generation — text
// ═════════════════════════════════════════════════════════════════════════════

std::string SlamProfiler::report_text(const SlamProfileSnapshot& snap) {
    std::ostringstream os;

    os << "═══════════════════════════════════════════════════════════════════\n";
    os << "  Thunderbird SLAM Performance Report\n";
    os << "  Uptime: " << snap.uptime_s << " s\n";
    os << "═══════════════════════════════════════════════════════════════════\n\n";

    // ── Latency table ───────────────────────────────────────────────────
    os << "  Latency Probes:\n";
    os << "  ─────────────────────────────────────────────────────────────────\n";

    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "  %-24s %8s %10s %10s %10s %10s %10s %10s\n",
        "Probe", "Count", "Mean(µs)", "Min(µs)", "Max(µs)",
        "P50(µs)", "P95(µs)", "P99(µs)");
    os << buf;

    os << "  ─────────────────────────────────────────────────────────────────\n";
    for (const auto& p : snap.probes) {
        std::snprintf(buf, sizeof(buf),
            "  %-24s %8lu %10.1f %10.1f %10.1f %10.1f %10.1f %10.1f\n",
            p.name.c_str(),
            static_cast<unsigned long>(p.count),
            p.mean_us, p.min_us, p.max_us,
            p.p50_us, p.p95_us, p.p99_us);
        os << buf;
    }

    // ── Memory ──────────────────────────────────────────────────────────
    os << "\n  Memory:\n";
    os << "  ─────────────────────────────────────────────────────────────────\n";
    std::snprintf(buf, sizeof(buf),
        "    RSS:           %.1f MB\n"
        "    Virtual:       %.1f MB\n"
        "    Map points:    %zu\n"
        "    Map memory:    %.1f MB\n"
        "    Ring buffers:  %.1f KB\n",
        snap.memory.rss_bytes / 1048576.0,
        snap.memory.vm_bytes / 1048576.0,
        snap.memory.map_points,
        snap.memory.map_bytes_estimate / 1048576.0,
        snap.memory.ring_buffer_bytes / 1024.0);
    os << buf;

    // ── CPU ─────────────────────────────────────────────────────────────
    os << "\n  CPU:\n";
    os << "  ─────────────────────────────────────────────────────────────────\n";
    std::snprintf(buf, sizeof(buf),
        "    Process CPU:     %.1f%%\n"
        "    Worker thread:   %.1f%%\n"
        "    User time:       %.2f s\n"
        "    System time:     %.2f s\n"
        "    Wall time:       %.2f s\n",
        snap.cpu.process_cpu_pct,
        snap.cpu.worker_thread_cpu_pct,
        snap.cpu.user_time_s,
        snap.cpu.system_time_s,
        snap.cpu.wall_time_s);
    os << buf;

    // ── Drift ───────────────────────────────────────────────────────────
    if (snap.drift.num_ground_truth_points > 0) {
        os << "\n  Drift:\n";
        os << "  ─────────────────────────────────────────────────────────────────\n";
        std::snprintf(buf, sizeof(buf),
            "    Distance:      %.1f m\n"
            "    ATE (RMSE):    %.3f m\n"
            "    Drift:         %.2f%%\n"
            "    Max error:     %.3f m\n"
            "    Mean rot err:  %.3f°\n"
            "    GT points:     %zu\n",
            snap.drift.distance_traveled_m,
            snap.drift.absolute_position_error_m,
            snap.drift.relative_drift_pct,
            snap.drift.max_position_error_m,
            snap.drift.mean_rotation_error_deg,
            snap.drift.num_ground_truth_points);
        os << buf;
    }

    os << "\n═══════════════════════════════════════════════════════════════════\n";
    return os.str();
}

// ═════════════════════════════════════════════════════════════════════════════
//  Benchmark plan factories
// ═════════════════════════════════════════════════════════════════════════════

BenchmarkPlan benchmark_plan_drone() {
    BenchmarkPlan plan;
    plan.name = "drone_standard";

    // 1. Static hover — zero drift baseline, memory & CPU stability.
    plan.scenarios.push_back({
        .name                   = "hover_static",
        .dataset_path           = "benchmark/drone/hover_static.tbrec",
        .ground_truth_path      = "benchmark/drone/hover_static_gt.txt",
        .imu_rate_hz            = 400.0,
        .lidar_rate_hz          = 10.0,
        .points_per_scan        = 10000,
        .expected_distance_m    = 0.0,
        .duration_s             = 30.0,
        .max_end_to_end_ms      = 30.0,
        .max_imu_propagation_us = 5.0,
        .max_esikf_update_ms    = 15.0,
        .max_memory_mb          = 200.0,
        .max_cpu_pct            = 80.0,
        .max_drift_pct          = 100.0,   // N/A for static
    });

    // 2. Figure-eight — moderate dynamics, good features.
    plan.scenarios.push_back({
        .name                   = "figure_eight_100m",
        .dataset_path           = "benchmark/drone/figure_eight_100m.tbrec",
        .ground_truth_path      = "benchmark/drone/figure_eight_100m_gt.txt",
        .imu_rate_hz            = 400.0,
        .lidar_rate_hz          = 10.0,
        .points_per_scan        = 15000,
        .expected_distance_m    = 100.0,
        .duration_s             = 60.0,
        .max_end_to_end_ms      = 30.0,
        .max_imu_propagation_us = 5.0,
        .max_esikf_update_ms    = 15.0,
        .max_memory_mb          = 200.0,
        .max_cpu_pct            = 80.0,
        .max_drift_pct          = 0.5,
    });

    // 3. Urban fast flight — high speed, rich geometry.
    plan.scenarios.push_back({
        .name                   = "urban_fast_500m",
        .dataset_path           = "benchmark/drone/urban_fast_500m.tbrec",
        .ground_truth_path      = "benchmark/drone/urban_fast_500m_gt.txt",
        .imu_rate_hz            = 400.0,
        .lidar_rate_hz          = 10.0,
        .points_per_scan        = 20000,
        .expected_distance_m    = 500.0,
        .duration_s             = 120.0,
        .max_end_to_end_ms      = 30.0,
        .max_imu_propagation_us = 5.0,
        .max_esikf_update_ms    = 15.0,
        .max_memory_mb          = 200.0,
        .max_cpu_pct            = 80.0,
        .max_drift_pct          = 1.0,
    });

    // 4. Long-range — 1 km, the primary drift benchmark.
    plan.scenarios.push_back({
        .name                   = "long_range_1km",
        .dataset_path           = "benchmark/drone/long_range_1km.tbrec",
        .ground_truth_path      = "benchmark/drone/long_range_1km_gt.txt",
        .imu_rate_hz            = 400.0,
        .lidar_rate_hz          = 10.0,
        .points_per_scan        = 20000,
        .expected_distance_m    = 1000.0,
        .duration_s             = 300.0,
        .max_end_to_end_ms      = 30.0,
        .max_imu_propagation_us = 5.0,
        .max_esikf_update_ms    = 15.0,
        .max_memory_mb          = 200.0,
        .max_cpu_pct            = 80.0,
        .max_drift_pct          = 1.0,
    });

    // 5. Aggressive yaw — high angular rate, tests IMU preintegration.
    plan.scenarios.push_back({
        .name                   = "aggressive_yaw_200m",
        .dataset_path           = "benchmark/drone/aggressive_yaw_200m.tbrec",
        .ground_truth_path      = "benchmark/drone/aggressive_yaw_200m_gt.txt",
        .imu_rate_hz            = 1000.0,
        .lidar_rate_hz          = 10.0,
        .points_per_scan        = 10000,
        .expected_distance_m    = 200.0,
        .duration_s             = 60.0,
        .max_end_to_end_ms      = 30.0,
        .max_imu_propagation_us = 5.0,
        .max_esikf_update_ms    = 15.0,
        .max_memory_mb          = 200.0,
        .max_cpu_pct            = 80.0,
        .max_drift_pct          = 1.5,
    });

    // 6. Degenerate corridor — narrow geometry, few features.
    plan.scenarios.push_back({
        .name                   = "degenerate_hall_300m",
        .dataset_path           = "benchmark/drone/degenerate_hall_300m.tbrec",
        .ground_truth_path      = "benchmark/drone/degenerate_hall_300m_gt.txt",
        .imu_rate_hz            = 400.0,
        .lidar_rate_hz          = 10.0,
        .points_per_scan        = 5000,
        .expected_distance_m    = 300.0,
        .duration_s             = 120.0,
        .max_end_to_end_ms      = 30.0,
        .max_imu_propagation_us = 5.0,
        .max_esikf_update_ms    = 15.0,
        .max_memory_mb          = 200.0,
        .max_cpu_pct            = 80.0,
        .max_drift_pct          = 2.0,
    });

    return plan;
}

BenchmarkPlan benchmark_plan_ground_vehicle() {
    BenchmarkPlan plan;
    plan.name = "ground_vehicle_standard";

    // 1. Parking lot — tight turns, loop closure opportunity.
    plan.scenarios.push_back({
        .name                   = "parking_lot_200m",
        .dataset_path           = "benchmark/ground/parking_lot_200m.tbrec",
        .ground_truth_path      = "benchmark/ground/parking_lot_200m_gt.txt",
        .imu_rate_hz            = 200.0,
        .lidar_rate_hz          = 10.0,
        .points_per_scan        = 30000,
        .expected_distance_m    = 200.0,
        .duration_s             = 60.0,
        .max_end_to_end_ms      = 15.0,
        .max_imu_propagation_us = 2.0,
        .max_esikf_update_ms    = 8.0,
        .max_memory_mb          = 400.0,
        .max_cpu_pct            = 50.0,
        .max_drift_pct          = 0.3,
    });

    // 2. Campus loop — mixed indoor/outdoor, pedestrians.
    plan.scenarios.push_back({
        .name                   = "campus_loop_1km",
        .dataset_path           = "benchmark/ground/campus_loop_1km.tbrec",
        .ground_truth_path      = "benchmark/ground/campus_loop_1km_gt.txt",
        .imu_rate_hz            = 200.0,
        .lidar_rate_hz          = 10.0,
        .points_per_scan        = 30000,
        .expected_distance_m    = 1000.0,
        .duration_s             = 200.0,
        .max_end_to_end_ms      = 15.0,
        .max_imu_propagation_us = 2.0,
        .max_esikf_update_ms    = 8.0,
        .max_memory_mb          = 400.0,
        .max_cpu_pct            = 50.0,
        .max_drift_pct          = 0.5,
    });

    // 3. Highway — high speed, sparse features at distance.
    plan.scenarios.push_back({
        .name                   = "highway_2km",
        .dataset_path           = "benchmark/ground/highway_2km.tbrec",
        .ground_truth_path      = "benchmark/ground/highway_2km_gt.txt",
        .imu_rate_hz            = 200.0,
        .lidar_rate_hz          = 10.0,
        .points_per_scan        = 40000,
        .expected_distance_m    = 2000.0,
        .duration_s             = 300.0,
        .max_end_to_end_ms      = 15.0,
        .max_imu_propagation_us = 2.0,
        .max_esikf_update_ms    = 8.0,
        .max_memory_mb          = 400.0,
        .max_cpu_pct            = 50.0,
        .max_drift_pct          = 0.5,
    });

    // 4. Dense urban — many dynamic objects, occlusion.
    plan.scenarios.push_back({
        .name                   = "urban_dense_1km",
        .dataset_path           = "benchmark/ground/urban_dense_1km.tbrec",
        .ground_truth_path      = "benchmark/ground/urban_dense_1km_gt.txt",
        .imu_rate_hz            = 200.0,
        .lidar_rate_hz          = 10.0,
        .points_per_scan        = 50000,
        .expected_distance_m    = 1000.0,
        .duration_s             = 200.0,
        .max_end_to_end_ms      = 15.0,
        .max_imu_propagation_us = 2.0,
        .max_esikf_update_ms    = 10.0,
        .max_memory_mb          = 400.0,
        .max_cpu_pct            = 50.0,
        .max_drift_pct          = 0.5,
    });

    // 5. Forest trail — unstructured geometry, bumpy terrain.
    plan.scenarios.push_back({
        .name                   = "forest_trail_500m",
        .dataset_path           = "benchmark/ground/forest_trail_500m.tbrec",
        .ground_truth_path      = "benchmark/ground/forest_trail_500m_gt.txt",
        .imu_rate_hz            = 200.0,
        .lidar_rate_hz          = 10.0,
        .points_per_scan        = 20000,
        .expected_distance_m    = 500.0,
        .duration_s             = 150.0,
        .max_end_to_end_ms      = 15.0,
        .max_imu_propagation_us = 2.0,
        .max_esikf_update_ms    = 8.0,
        .max_memory_mb          = 400.0,
        .max_cpu_pct            = 50.0,
        .max_drift_pct          = 0.8,
    });

    // 6. Multi-story garage — vertical motion, poor GPS.
    plan.scenarios.push_back({
        .name                   = "garage_multi_300m",
        .dataset_path           = "benchmark/ground/garage_multi_300m.tbrec",
        .ground_truth_path      = "benchmark/ground/garage_multi_300m_gt.txt",
        .imu_rate_hz            = 200.0,
        .lidar_rate_hz          = 10.0,
        .points_per_scan        = 25000,
        .expected_distance_m    = 300.0,
        .duration_s             = 120.0,
        .max_end_to_end_ms      = 15.0,
        .max_imu_propagation_us = 2.0,
        .max_esikf_update_ms    = 8.0,
        .max_memory_mb          = 400.0,
        .max_cpu_pct            = 50.0,
        .max_drift_pct          = 0.5,
    });

    return plan;
}

BenchmarkPlan benchmark_plan_stress() {
    BenchmarkPlan plan;
    plan.name = "stress_test";

    // 1. Max throughput — 100K points per scan.
    plan.scenarios.push_back({
        .name                   = "max_throughput_100k",
        .dataset_path           = "benchmark/stress/max_throughput.tbrec",
        .ground_truth_path      = "",
        .imu_rate_hz            = 400.0,
        .lidar_rate_hz          = 10.0,
        .points_per_scan        = 100000,
        .expected_distance_m    = 100.0,
        .duration_s             = 60.0,
        .max_end_to_end_ms      = 80.0,
        .max_imu_propagation_us = 5.0,
        .max_esikf_update_ms    = 50.0,
        .max_memory_mb          = 800.0,
        .max_cpu_pct            = 95.0,
        .max_drift_pct          = 100.0,  // accuracy not primary
    });

    // 2. Memory pressure — 1M map points.
    plan.scenarios.push_back({
        .name                   = "memory_pressure_1M",
        .dataset_path           = "benchmark/stress/memory_pressure.tbrec",
        .ground_truth_path      = "",
        .imu_rate_hz            = 400.0,
        .lidar_rate_hz          = 10.0,
        .points_per_scan        = 50000,
        .expected_distance_m    = 500.0,
        .duration_s             = 300.0,
        .max_end_to_end_ms      = 80.0,
        .max_imu_propagation_us = 5.0,
        .max_esikf_update_ms    = 50.0,
        .max_memory_mb          = 1500.0,
        .max_cpu_pct            = 95.0,
        .max_drift_pct          = 100.0,
    });

    // 3. Sustained 1-hour — leak detection.
    plan.scenarios.push_back({
        .name                   = "sustained_1hr",
        .dataset_path           = "benchmark/stress/sustained_1hr.tbrec",
        .ground_truth_path      = "",
        .imu_rate_hz            = 400.0,
        .lidar_rate_hz          = 10.0,
        .points_per_scan        = 20000,
        .expected_distance_m    = 5000.0,
        .duration_s             = 3600.0,
        .max_end_to_end_ms      = 30.0,
        .max_imu_propagation_us = 5.0,
        .max_esikf_update_ms    = 15.0,
        .max_memory_mb          = 400.0,
        .max_cpu_pct            = 80.0,
        .max_drift_pct          = 100.0,
    });

    return plan;
}

} // namespace thunderbird::odom
