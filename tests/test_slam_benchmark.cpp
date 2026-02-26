// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Benchmark Harness
// ─────────────────────────────────────────────────────────────────────────────
//
// Standalone benchmark executable that exercises the SLAM pipeline with
// synthetic or recorded data and validates against performance targets.
//
// Build:
//   cmake -DTHUNDERBIRD_BUILD_TESTS=ON -DTHUNDERBIRD_SLAM_PROFILING=ON ..
//   make test_slam_benchmark
//
// Run:
//   ./test_slam_benchmark                     # all synthetic benchmarks
//   ./test_slam_benchmark --scenario drone    # drone plan only
//   ./test_slam_benchmark --scenario ground   # ground vehicle only
//   ./test_slam_benchmark --scenario stress   # stress tests only
//   ./test_slam_benchmark --json report.json  # export JSON report
//
// ─────────────────────────────────────────────────────────────────────────────
//
// Benchmark Methodology
// ─────────────────────
//
// 1. SYNTHETIC DATA GENERATION
//    When no recorded dataset (.tbrec) is available, the harness generates
//    synthetic IMU + LiDAR data with a known ground-truth trajectory:
//
//      a) Trajectory: circular / figure-eight / linear ramp.
//      b) IMU: exact rotational velocity + acceleration, then add configurable
//         Gaussian noise matching the scenario's noise model.
//      c) LiDAR: random point cloud around the current position, with
//         realistic point density and timestamp offsets.
//      d) Ground truth: analytic position/quaternion at each IMU tick.
//
// 2. EXECUTION MODEL
//      a) Initialize AcmeSlamEngine with scenario config.
//      b) Feed all IMU + LiDAR data in real-time order (sleep optional).
//      c) Drain all SlamOutput via polling loop.
//      d) Record profiler probes throughout.
//      e) After completion, compute drift via profiler GT matching.
//
// 3. METRICS COLLECTED
//      - End-to-end latency (feed → output timestamp delta)
//      - Per-component timing (IMU propagation, deskew, ESIKF, ikd-Tree)
//      - Memory usage (RSS, map size)
//      - CPU load (process-level)
//      - Drift (ATE RMSE, relative %, max error, rotation error)
//
// 4. PASS / FAIL CRITERIA
//      Each metric is compared against BenchmarkScenario thresholds.
//      A scenario fails if ANY metric exceeds its threshold.
//      The overall benchmark fails if ANY scenario fails.
//
// ─────────────────────────────────────────────────────────────────────────────

// Force profiling ON for this translation unit.
#ifndef THUNDERBIRD_SLAM_PROFILING
#define THUNDERBIRD_SLAM_PROFILING 1
#endif

#include "thunderbird/odom/slam_profiler.h"
#include "thunderbird/odom/slam_engine.h"
#include "thunderbird/odom/slam_types.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <thread>
#include <vector>

namespace {

using namespace thunderbird::odom;

// ═════════════════════════════════════════════════════════════════════════════
//  Trajectory generators
// ═════════════════════════════════════════════════════════════════════════════

/// Circular trajectory: constant-altitude circle.
/// Returns position and quaternion at time t (seconds).
struct CircularTrajectory {
    double radius_m  = 10.0;
    double speed_mps = 2.0;
    double altitude  = 5.0;

    void at(double t, std::array<double,3>& pos,
            std::array<double,4>& quat) const {
        const double omega = speed_mps / radius_m;  // rad/s
        const double theta = omega * t;

        pos[0] = radius_m * std::cos(theta);
        pos[1] = radius_m * std::sin(theta);
        pos[2] = altitude;

        // Heading tangent to circle.
        const double yaw = theta + 3.14159265358979323846 / 2.0;
        // Quaternion from yaw only (rotation about Z).
        quat[0] = std::cos(yaw / 2.0);
        quat[1] = 0.0;
        quat[2] = 0.0;
        quat[3] = std::sin(yaw / 2.0);
    }

    /// Analytic velocity at time t.
    void velocity(double t, std::array<double,3>& vel) const {
        const double omega = speed_mps / radius_m;
        const double theta = omega * t;
        vel[0] = -radius_m * omega * std::sin(theta);
        vel[1] =  radius_m * omega * std::cos(theta);
        vel[2] = 0.0;
    }

    /// Analytic acceleration at time t (centripetal).
    void acceleration(double t, std::array<double,3>& acc) const {
        const double omega = speed_mps / radius_m;
        const double theta = omega * t;
        acc[0] = -radius_m * omega * omega * std::cos(theta);
        acc[1] = -radius_m * omega * omega * std::sin(theta);
        acc[2] = 9.81;  // gravity compensation
    }

    /// Angular velocity (rad/s).
    void gyro(double /*t*/, std::array<double,3>& gyr) const {
        const double omega = speed_mps / radius_m;
        gyr[0] = 0.0;
        gyr[1] = 0.0;
        gyr[2] = omega;
    }
};

/// Linear trajectory: straight line at constant speed.
struct LinearTrajectory {
    double speed_mps = 5.0;

    void at(double t, std::array<double,3>& pos,
            std::array<double,4>& quat) const {
        pos[0] = speed_mps * t;
        pos[1] = 0.0;
        pos[2] = 0.0;
        quat   = {1.0, 0.0, 0.0, 0.0};
    }

    void velocity(double /*t*/, std::array<double,3>& vel) const {
        vel = {speed_mps, 0.0, 0.0};
    }

    void acceleration(double /*t*/, std::array<double,3>& acc) const {
        acc = {0.0, 0.0, 9.81};
    }

    void gyro(double /*t*/, std::array<double,3>& gyr) const {
        gyr = {0.0, 0.0, 0.0};
    }
};

// ═════════════════════════════════════════════════════════════════════════════
//  Synthetic data generators
// ═════════════════════════════════════════════════════════════════════════════

/// Generate synthetic IMU samples along a trajectory.
template <typename Trajectory>
std::vector<ImuSample> generate_imu(
    const Trajectory& traj,
    double duration_s,
    double rate_hz,
    double accel_noise_sigma = 0.01,
    double gyro_noise_sigma  = 0.001)
{
    std::mt19937 rng(42);
    std::normal_distribution<double> accel_noise(0.0, accel_noise_sigma);
    std::normal_distribution<double> gyro_noise(0.0, gyro_noise_sigma);

    const double dt = 1.0 / rate_hz;
    const size_t n  = static_cast<size_t>(duration_s * rate_hz);

    std::vector<ImuSample> samples;
    samples.reserve(n);

    for (size_t i = 0; i < n; ++i) {
        const double t = static_cast<double>(i) * dt;
        ImuSample s;
        s.timestamp_ns = static_cast<int64_t>(t * 1.0e9);

        std::array<double,3> acc{}, gyr{};
        traj.acceleration(t, acc);
        traj.gyro(t, gyr);

        s.accel[0] = acc[0] + accel_noise(rng);
        s.accel[1] = acc[1] + accel_noise(rng);
        s.accel[2] = acc[2] + accel_noise(rng);
        s.gyro[0]  = gyr[0] + gyro_noise(rng);
        s.gyro[1]  = gyr[1] + gyro_noise(rng);
        s.gyro[2]  = gyr[2] + gyro_noise(rng);
        s.temperature = 25.0;

        samples.push_back(s);
    }
    return samples;
}

/// Generate synthetic LiDAR point clouds.
template <typename Trajectory>
std::vector<std::shared_ptr<const PointCloudFrame>> generate_lidar(
    const Trajectory& traj,
    double duration_s,
    double rate_hz,
    size_t points_per_scan)
{
    std::mt19937 rng(123);
    std::normal_distribution<double> noise(0.0, 0.02);  // 2 cm noise
    std::uniform_real_distribution<double> theta_dist(0.0, 2.0 * 3.14159265358979323846);
    std::uniform_real_distribution<double> phi_dist(-0.4, 0.4);  // ±23°
    std::uniform_real_distribution<double> range_dist(1.0, 30.0);

    const double dt = 1.0 / rate_hz;
    const size_t n  = static_cast<size_t>(duration_s * rate_hz);

    std::vector<std::shared_ptr<const PointCloudFrame>> clouds;
    clouds.reserve(n);

    for (size_t i = 0; i < n; ++i) {
        const double t = static_cast<double>(i) * dt;
        auto cloud = std::make_shared<PointCloudFrame>();
        cloud->timestamp_ns = static_cast<int64_t>(t * 1.0e9);
        cloud->sequence     = static_cast<uint32_t>(i);
        cloud->is_deskewed  = false;
        cloud->points.resize(points_per_scan);

        std::array<double,3> pos{};
        std::array<double,4> quat{};
        traj.at(t, pos, quat);

        const double scan_dt_ns = dt * 1.0e9;
        for (size_t j = 0; j < points_per_scan; ++j) {
            auto& pt = cloud->points[j];
            const double th = theta_dist(rng);
            const double ph = phi_dist(rng);
            const double r  = range_dist(rng);

            pt.x = static_cast<float>(r * std::cos(ph) * std::cos(th) + noise(rng));
            pt.y = static_cast<float>(r * std::cos(ph) * std::sin(th) + noise(rng));
            pt.z = static_cast<float>(r * std::sin(ph) + noise(rng));
            pt.intensity = 100.0f;
            pt.dt_ns = static_cast<int32_t>(
                scan_dt_ns * static_cast<double>(j) / static_cast<double>(points_per_scan));
        }

        clouds.push_back(std::move(cloud));
    }
    return clouds;
}

/// Generate ground truth trajectory.
template <typename Trajectory>
std::vector<GroundTruthPose> generate_ground_truth(
    const Trajectory& traj,
    double duration_s,
    double rate_hz)
{
    const double dt = 1.0 / rate_hz;
    const size_t n  = static_cast<size_t>(duration_s * rate_hz);

    std::vector<GroundTruthPose> gt;
    gt.reserve(n);

    for (size_t i = 0; i < n; ++i) {
        const double t = static_cast<double>(i) * dt;
        GroundTruthPose gtp;
        gtp.timestamp_ns = static_cast<int64_t>(t * 1.0e9);
        traj.at(t, gtp.position, gtp.quaternion);
        gt.push_back(gtp);
    }
    return gt;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Benchmark execution
// ═════════════════════════════════════════════════════════════════════════════

struct BenchmarkResult {
    std::string scenario_name;
    bool        passed{true};
    ProfileSnapshot profile;

    // Per-metric pass/fail.
    bool latency_ok{true};
    bool imu_time_ok{true};
    bool esikf_time_ok{true};
    bool memory_ok{true};
    bool cpu_ok{true};
    bool drift_ok{true};

    std::string failure_reason;
};

/// Run a single synthetic benchmark scenario.
BenchmarkResult run_synthetic_benchmark(const BenchmarkScenario& scenario) {
    BenchmarkResult result;
    result.scenario_name = scenario.name;

    auto& prof = SlamProfiler::instance();
    prof.reset();

    // ── Configure engine ────────────────────────────────────────────────
    SlamEngineConfig config;
    config.imu_rate_hz   = scenario.imu_rate_hz;
    config.lidar_rate_hz = scenario.lidar_rate_hz;
    config.publish_propagated_poses = true;

    AcmeSlamEngine engine;
    if (!engine.initialize(config)) {
        result.passed = false;
        result.failure_reason = "Engine initialization failed";
        return result;
    }

    // ── Generate data ───────────────────────────────────────────────────
    const double duration = scenario.duration_s > 0.0 ? scenario.duration_s : 30.0;

    CircularTrajectory traj;
    if (scenario.expected_distance_m > 0) {
        // Adjust speed so total distance matches.
        traj.speed_mps = scenario.expected_distance_m / duration;
    }

    auto imu_samples = generate_imu(
        traj, duration, scenario.imu_rate_hz,
        0.01, 0.001);

    auto lidar_clouds = generate_lidar(
        traj, duration, scenario.lidar_rate_hz,
        scenario.points_per_scan);

    auto ground_truth = generate_ground_truth(
        traj, duration, scenario.lidar_rate_hz);

    // Feed ground truth to profiler.
    for (const auto& gt : ground_truth) {
        prof.feed_ground_truth(gt);
    }

    // ── Feed data to engine ─────────────────────────────────────────────
    size_t imu_idx   = 0;
    size_t lidar_idx = 0;
    std::vector<SlamOutput> outputs;

    while (imu_idx < imu_samples.size() || lidar_idx < lidar_clouds.size()) {
        // Interleave IMU and LiDAR by timestamp.
        bool feed_imu = false;
        if (imu_idx < imu_samples.size() && lidar_idx < lidar_clouds.size()) {
            feed_imu = imu_samples[imu_idx].timestamp_ns <=
                       lidar_clouds[lidar_idx]->timestamp_ns;
        } else {
            feed_imu = imu_idx < imu_samples.size();
        }

        if (feed_imu) {
            auto t0 = profiler_now_ns();
            engine.feedImu(imu_samples[imu_idx]);
            SLAM_PROFILE_RECORD("feed_imu", profiler_now_ns() - t0);
            ++imu_idx;
        } else {
            auto t0 = profiler_now_ns();
            engine.feedPointCloud(lidar_clouds[lidar_idx]);
            SLAM_PROFILE_RECORD("feed_lidar", profiler_now_ns() - t0);
            ++lidar_idx;
        }

        // Drain outputs.
        SlamOutput out;
        while (engine.getLatestOutput(out)) {
            // Record estimated pose for drift evaluation.
            prof.feed_estimated_pose(
                out.pose.timestamp_ns,
                out.pose.position,
                out.pose.quaternion);

            SLAM_PROFILE_RECORD("end_to_end",
                profiler_now_ns() - out.pose.timestamp_ns);

            outputs.push_back(std::move(out));
        }
    }

    // Wait for pipeline to flush (max 2 s).
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
        SlamOutput out;
        if (engine.getLatestOutput(out)) {
            prof.feed_estimated_pose(
                out.pose.timestamp_ns,
                out.pose.position,
                out.pose.quaternion);
            outputs.push_back(std::move(out));
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    engine.shutdown();

    // ── Compute drift ───────────────────────────────────────────────────
    prof.compute_drift();

    // ── Take snapshot ───────────────────────────────────────────────────
    result.profile = prof.snapshot();

    // ── Evaluate pass/fail ──────────────────────────────────────────────
    auto find_probe = [&](const std::string& name) -> const ProbeSnapshot* {
        for (const auto& p : result.profile.probes) {
            if (p.name == name) return &p;
        }
        return nullptr;
    };

    // End-to-end latency.
    if (auto* p = find_probe("end_to_end")) {
        const double e2e_ms = p->p95_us / 1000.0;
        if (e2e_ms > scenario.max_end_to_end_ms) {
            result.latency_ok = false;
            result.passed = false;
            char buf[128];
            std::snprintf(buf, sizeof(buf),
                "E2E latency P95 = %.1f ms > %.1f ms threshold; ",
                e2e_ms, scenario.max_end_to_end_ms);
            result.failure_reason += buf;
        }
    }

    // IMU propagation.
    if (auto* p = find_probe("imu_propagate")) {
        if (p->mean_us > scenario.max_imu_propagation_us) {
            result.imu_time_ok = false;
            result.passed = false;
            char buf[128];
            std::snprintf(buf, sizeof(buf),
                "IMU propagation mean = %.1f µs > %.1f µs; ",
                p->mean_us, scenario.max_imu_propagation_us);
            result.failure_reason += buf;
        }
    }

    // ESIKF update.
    if (auto* p = find_probe("esikf_update")) {
        const double esikf_ms = p->mean_us / 1000.0;
        if (esikf_ms > scenario.max_esikf_update_ms) {
            result.esikf_time_ok = false;
            result.passed = false;
            char buf[128];
            std::snprintf(buf, sizeof(buf),
                "ESIKF update mean = %.1f ms > %.1f ms; ",
                esikf_ms, scenario.max_esikf_update_ms);
            result.failure_reason += buf;
        }
    }

    // Memory.
    const double rss_mb = result.profile.memory.rss_bytes / 1048576.0;
    if (rss_mb > scenario.max_memory_mb) {
        result.memory_ok = false;
        result.passed = false;
        char buf[128];
        std::snprintf(buf, sizeof(buf),
            "RSS = %.1f MB > %.1f MB; ", rss_mb, scenario.max_memory_mb);
        result.failure_reason += buf;
    }

    // CPU.
    if (result.profile.cpu.process_cpu_pct > scenario.max_cpu_pct) {
        result.cpu_ok = false;
        result.passed = false;
        char buf[128];
        std::snprintf(buf, sizeof(buf),
            "CPU = %.1f%% > %.1f%%; ",
            result.profile.cpu.process_cpu_pct, scenario.max_cpu_pct);
        result.failure_reason += buf;
    }

    // Drift.
    if (scenario.max_drift_pct < 100.0 &&
        result.profile.drift.relative_drift_pct > scenario.max_drift_pct) {
        result.drift_ok = false;
        result.passed = false;
        char buf[128];
        std::snprintf(buf, sizeof(buf),
            "Drift = %.2f%% > %.2f%%; ",
            result.profile.drift.relative_drift_pct, scenario.max_drift_pct);
        result.failure_reason += buf;
    }

    return result;
}

/// Run all scenarios in a BenchmarkPlan.
std::vector<BenchmarkResult> run_benchmark_plan(const BenchmarkPlan& plan) {
    std::vector<BenchmarkResult> results;
    results.reserve(plan.scenarios.size());

    std::printf("\n  Running benchmark plan: %s (%zu scenarios)\n",
                plan.name.c_str(), plan.scenarios.size());
    std::printf("  ─────────────────────────────────────────────────────────\n");

    for (const auto& scenario : plan.scenarios) {
        std::printf("  [RUN ] %s ...\n", scenario.name.c_str());
        auto result = run_synthetic_benchmark(scenario);

        if (result.passed) {
            std::printf("  [PASS] %s\n", scenario.name.c_str());
        } else {
            std::printf("  [FAIL] %s: %s\n",
                        scenario.name.c_str(),
                        result.failure_reason.c_str());
        }

        results.push_back(std::move(result));
    }

    return results;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Micro-benchmarks (no engine, just component timing)
// ═════════════════════════════════════════════════════════════════════════════

void microbench_profiler_overhead() {
    std::printf("\n  Micro-benchmark: profiler overhead\n");
    std::printf("  ─────────────────────────────────────────────────────────\n");

    auto& prof = SlamProfiler::instance();
    prof.reset();
    auto* p = prof.probe("overhead_test");

    // Warm up.
    for (int i = 0; i < 1000; ++i) {
        auto t0 = profiler_now_ns();
        auto t1 = profiler_now_ns();
        p->record(t1 - t0);
    }
    p->reset();

    // Measure.
    constexpr int N = 100000;
    auto wall_start = profiler_now_ns();
    for (int i = 0; i < N; ++i) {
        SLAM_PROFILE_SCOPE("overhead_test");
    }
    auto wall_end = profiler_now_ns();

    const double per_call_ns = static_cast<double>(wall_end - wall_start)
                             / static_cast<double>(N);
    std::printf("    profiler_now_ns() call:  %lu calls, %.1f ns/call (wall)\n",
                static_cast<unsigned long>(N), per_call_ns);
    std::printf("    SLAM_PROFILE_SCOPE overhead: %.1f ns/scope\n", per_call_ns);

    // Scoped timer: should report ~same as wall measurement divided by N.
    std::printf("    Probe mean (recorded):   %.1f ns\n", p->mean_ns());
    std::printf("    Expected: < 50 ns per scope on modern x86\n\n");
}

void microbench_histogram_accuracy() {
    std::printf("  Micro-benchmark: histogram accuracy\n");
    std::printf("  ─────────────────────────────────────────────────────────\n");

    auto& prof = SlamProfiler::instance();
    prof.reset();
    auto* p = prof.probe("histogram_test");

    // Inject known durations: 1µs, 10µs, 100µs, 1ms, 10ms.
    auto inject = [&](int64_t ns, int count) {
        for (int i = 0; i < count; ++i) p->record(ns);
    };

    inject(1'000, 100);       // 1 µs → bucket 1
    inject(10'000, 200);      // 10 µs → bucket ~4
    inject(100'000, 300);     // 100 µs → bucket ~7
    inject(1'000'000, 400);   // 1 ms → bucket ~10
    inject(10'000'000, 500);  // 10 ms → bucket ~14

    std::printf("    Injected: 100×1µs, 200×10µs, 300×100µs, 400×1ms, 500×10ms\n");
    std::printf("    Total count: %lu (expected 1500)\n",
                static_cast<unsigned long>(p->count));
    std::printf("    P50 (µs):  %.0f (expect ~100 µs range)\n", p->percentile_us(50.0));
    std::printf("    P95 (µs):  %.0f (expect ~10000 µs range)\n", p->percentile_us(95.0));
    std::printf("    P99 (µs):  %.0f (expect ~10000 µs range)\n\n", p->percentile_us(99.0));
}

void microbench_imu_sample_throughput() {
    std::printf("  Micro-benchmark: ImuSample feed throughput\n");
    std::printf("  ─────────────────────────────────────────────────────────\n");

    AcmeSlamEngine engine;
    SlamEngineConfig config;
    config.imu_rate_hz = 1000.0;
    if (!engine.initialize(config)) {
        std::printf("    [SKIP] engine init failed\n\n");
        return;
    }

    constexpr int N = 100000;
    ImuSample sample{};
    sample.accel = {0.0, 0.0, 9.81};
    sample.temperature = 25.0;

    auto start = profiler_now_ns();
    for (int i = 0; i < N; ++i) {
        sample.timestamp_ns = static_cast<int64_t>(i) * 1'000'000; // 1 ms apart
        engine.feedImu(sample);
    }
    auto elapsed = profiler_now_ns() - start;

    engine.shutdown();

    const double per_call_ns = static_cast<double>(elapsed) / N;
    const double throughput_mhz = 1.0e9 / per_call_ns / 1.0e6;

    std::printf("    %d feedImu() calls: %.1f ns/call (%.2f MHz)\n",
                N, per_call_ns, throughput_mhz);
    std::printf("    Target: < 50 ns/call (> 20 MHz)\n\n");
}

void microbench_snapshot_cost() {
    std::printf("  Micro-benchmark: snapshot() cost\n");
    std::printf("  ─────────────────────────────────────────────────────────\n");

    auto& prof = SlamProfiler::instance();
    prof.reset();

    // Register 16 probes with some data.
    for (int i = 0; i < 16; ++i) {
        char name[32];
        std::snprintf(name, sizeof(name), "bench_probe_%02d", i);
        auto* p = prof.probe(name);
        for (int j = 0; j < 1000; ++j) p->record(j * 100);
    }

    constexpr int N = 10000;
    auto start = profiler_now_ns();
    for (int i = 0; i < N; ++i) {
        auto snap = prof.snapshot();
        (void)snap;
    }
    auto elapsed = profiler_now_ns() - start;

    std::printf("    %d snapshot() calls (16 probes): %.1f µs/call\n",
                N, static_cast<double>(elapsed) / N / 1000.0);
    std::printf("    Target: < 10 µs per snapshot\n\n");
}

void microbench_report_generation() {
    std::printf("  Micro-benchmark: report generation\n");
    std::printf("  ─────────────────────────────────────────────────────────\n");

    auto& prof = SlamProfiler::instance();
    // Use existing probes from snapshot_cost test.
    auto snap = prof.snapshot();

    constexpr int N = 1000;

    auto start_json = profiler_now_ns();
    for (int i = 0; i < N; ++i) {
        auto json = SlamProfiler::report_json(snap);
        (void)json;
    }
    auto json_ns = profiler_now_ns() - start_json;

    auto start_csv = profiler_now_ns();
    for (int i = 0; i < N; ++i) {
        auto csv = SlamProfiler::report_csv(snap);
        (void)csv;
    }
    auto csv_ns = profiler_now_ns() - start_csv;

    auto start_text = profiler_now_ns();
    for (int i = 0; i < N; ++i) {
        auto text = SlamProfiler::report_text(snap);
        (void)text;
    }
    auto text_ns = profiler_now_ns() - start_text;

    std::printf("    JSON:  %.1f µs/call\n", static_cast<double>(json_ns) / N / 1000.0);
    std::printf("    CSV:   %.1f µs/call\n", static_cast<double>(csv_ns)  / N / 1000.0);
    std::printf("    Text:  %.1f µs/call\n\n", static_cast<double>(text_ns) / N / 1000.0);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Main: unit tests + benchmarks
// ═════════════════════════════════════════════════════════════════════════════

/// Unit tests for the profiler infrastructure.
int run_unit_tests() {
    int failures = 0;

    std::printf("\n═══════════════════════════════════════════════════════════════\n");
    std::printf("  Profiler Unit Tests\n");
    std::printf("═══════════════════════════════════════════════════════════════\n\n");

    auto& prof = SlamProfiler::instance();

    // ── Test 1: probe registration ──────────────────────────────────────
    {
        prof.reset();
        auto* p1 = prof.probe("test_probe_1");
        auto* p2 = prof.probe("test_probe_2");
        auto* p1_again = prof.probe("test_probe_1");

        bool ok = (p1 != nullptr && p2 != nullptr && p1 == p1_again &&
                   prof.probe_count() == 2);
        std::printf("  [%s] Probe registration & dedup\n", ok ? "PASS" : "FAIL");
        if (!ok) ++failures;
    }

    // ── Test 2: record & read back ──────────────────────────────────────
    {
        prof.reset();
        auto* p = prof.probe("test_record");
        p->record(1000);   // 1 µs
        p->record(2000);   // 2 µs
        p->record(3000);   // 3 µs

        bool ok = (p->count == 3 &&
                   p->sum_ns == 6000 &&
                   p->min_ns == 1000 &&
                   p->max_ns == 3000 &&
                   std::abs(p->mean_ns() - 2000.0) < 1.0);
        std::printf("  [%s] Record and read back\n", ok ? "PASS" : "FAIL");
        if (!ok) ++failures;
    }

    // ── Test 3: histogram placement ─────────────────────────────────────
    {
        prof.reset();
        auto* p = prof.probe("test_histo");
        // 500 ns → bucket 0 (< 1 µs)
        p->record(500);
        // 1500 ns = 1.5 µs → bucket 1 [1, 2) µs
        p->record(1500);
        // 10 ms = 10000 µs → bucket ~14
        p->record(10'000'000);

        bool ok = (p->histogram[0] == 1 &&  // < 1 µs
                   p->count == 3);
        std::printf("  [%s] Histogram bucket placement\n", ok ? "PASS" : "FAIL");
        if (!ok) ++failures;
    }

    // ── Test 4: atomic recording ────────────────────────────────────────
    {
        prof.reset();
        auto* p = prof.probe("test_atomic");
        p->record_atomic(5000);
        p->record_atomic(5000);

        bool ok = (p->atomic_count.load() == 2 &&
                   p->atomic_sum_ns.load() == 10000);
        std::printf("  [%s] Atomic recording\n", ok ? "PASS" : "FAIL");
        if (!ok) ++failures;
    }

    // ── Test 5: enable/disable ──────────────────────────────────────────
    {
        prof.reset();
        auto* p = prof.probe("test_toggle");
        p->record(1000);
        prof.disable("test_toggle");
        // Direct record still works (macro would skip).
        // The ScopedTimer checks enabled flag.
        bool ok = (p->enabled.load() == false);
        prof.enable("test_toggle");
        ok = ok && (p->enabled.load() == true);
        std::printf("  [%s] Enable/disable toggle\n", ok ? "PASS" : "FAIL");
        if (!ok) ++failures;
    }

    // ── Test 6: snapshot ────────────────────────────────────────────────
    {
        prof.reset();
        auto* p = prof.probe("test_snap");
        p->record(5000);
        p->record(15000);
        auto snap = prof.snapshot();

        // Note: reset() clears data but does NOT unregister probes, so
        // snap.probes includes probes from earlier tests.  Find ours.
        const ProbeSnapshot* found = nullptr;
        for (const auto& ps : snap.probes) {
            if (ps.name == "test_snap") { found = &ps; break; }
        }
        bool ok = (found != nullptr &&
                   found->count == 2 &&
                   snap.probes.size() >= 1 &&
                   snap.uptime_s >= 0.0);
        std::printf("  [%s] Snapshot capture\n", ok ? "PASS" : "FAIL");
        if (!ok) ++failures;
    }

    // ── Test 7: drift computation ───────────────────────────────────────
    {
        prof.reset();
        // Feed matching GT and estimated at same positions → zero drift.
        for (int i = 0; i < 100; ++i) {
            int64_t ts = static_cast<int64_t>(i) * 100'000'000; // 100 ms
            double x = static_cast<double>(i) * 0.1;
            GroundTruthPose gt;
            gt.timestamp_ns = ts;
            gt.position = {x, 0.0, 0.0};
            prof.feed_ground_truth(gt);
            prof.feed_estimated_pose(ts, {x, 0.0, 0.0}, {1,0,0,0});
        }
        prof.compute_drift();
        auto snap = prof.snapshot();

        bool ok = (snap.drift.absolute_position_error_m < 0.001 &&
                   snap.drift.distance_traveled_m > 0.0);
        std::printf("  [%s] Drift computation (zero-error case)\n", ok ? "PASS" : "FAIL");
        if (!ok) ++failures;
    }

    // ── Test 8: drift with known error ──────────────────────────────────
    {
        prof.reset();
        // Estimated drifts 1 m over 100 m trajectory.
        for (int i = 0; i < 100; ++i) {
            int64_t ts = static_cast<int64_t>(i) * 100'000'000;
            double x = static_cast<double>(i) * 1.0;
            double drift = static_cast<double>(i) * 0.01; // 1% drift
            GroundTruthPose gt;
            gt.timestamp_ns = ts;
            gt.position = {x, 0.0, 0.0};
            prof.feed_ground_truth(gt);
            prof.feed_estimated_pose(ts, {x + drift, 0.0, 0.0}, {1,0,0,0});
        }
        prof.compute_drift();
        auto snap = prof.snapshot();

        bool ok = (snap.drift.absolute_position_error_m > 0.0 &&
                   snap.drift.relative_drift_pct > 0.0);
        std::printf("  [%s] Drift computation (known-error case: %.2f%%)\n",
                    ok ? "PASS" : "FAIL", snap.drift.relative_drift_pct);
        if (!ok) ++failures;
    }

    // ── Test 9: report generation ───────────────────────────────────────
    {
        prof.reset();
        prof.probe("test_report")->record(10000);
        auto snap = prof.snapshot();
        auto json = SlamProfiler::report_json(snap);
        auto csv  = SlamProfiler::report_csv(snap);
        auto text = SlamProfiler::report_text(snap);

        bool ok = (!json.empty() && json.find("test_report") != std::string::npos &&
                   !csv.empty() && csv.find("test_report") != std::string::npos &&
                   !text.empty() && text.find("test_report") != std::string::npos);
        std::printf("  [%s] Report generation (JSON=%zu, CSV=%zu, Text=%zu bytes)\n",
                    ok ? "PASS" : "FAIL", json.size(), csv.size(), text.size());
        if (!ok) ++failures;
    }

    // ── Test 10: benchmark plan factories ───────────────────────────────
    {
        auto drone   = benchmark_plan_drone();
        auto ground  = benchmark_plan_ground_vehicle();
        auto stress  = benchmark_plan_stress();

        bool ok = (drone.scenarios.size() == 6 &&
                   ground.scenarios.size() == 6 &&
                   stress.scenarios.size() == 3);
        std::printf("  [%s] Benchmark plan factories (drone=%zu, ground=%zu, stress=%zu)\n",
                    ok ? "PASS" : "FAIL",
                    drone.scenarios.size(), ground.scenarios.size(),
                    stress.scenarios.size());
        if (!ok) ++failures;
    }

    std::printf("\n  %s: %d test%s\n",
                failures == 0 ? "ALL PASSED" : "FAILURES",
                10, failures == 1 ? " failed" : "");

    return failures;
}

} // anonymous namespace

// ═════════════════════════════════════════════════════════════════════════════
//  Entry point
// ═════════════════════════════════════════════════════════════════════════════

int main(int argc, char* argv[]) {
    // ── Parse arguments ─────────────────────────────────────────────────
    std::string scenario_filter;
    std::string json_output_path;
    bool run_microbench = true;
    bool run_plans      = true;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--scenario") == 0 && i + 1 < argc) {
            scenario_filter = argv[++i];
        } else if (std::strcmp(argv[i], "--json") == 0 && i + 1 < argc) {
            json_output_path = argv[++i];
        } else if (std::strcmp(argv[i], "--no-plans") == 0) {
            run_plans = false;
        } else if (std::strcmp(argv[i], "--no-micro") == 0) {
            run_microbench = false;
        } else if (std::strcmp(argv[i], "-h") == 0 ||
                   std::strcmp(argv[i], "--help") == 0) {
            std::printf(
                "Usage: test_slam_benchmark [OPTIONS]\n"
                "  --scenario <name>  Run only drone|ground|stress plans\n"
                "  --json <path>      Export JSON report\n"
                "  --no-plans         Skip full benchmark plans\n"
                "  --no-micro         Skip micro-benchmarks\n"
                "  -h, --help         Show this help\n");
            return 0;
        }
    }

    int total_failures = 0;

    // ── Unit tests (always run) ─────────────────────────────────────────
    total_failures += run_unit_tests();

    // ── Micro-benchmarks ────────────────────────────────────────────────
    if (run_microbench) {
        std::printf("\n═══════════════════════════════════════════════════════════════\n");
        std::printf("  Micro-Benchmarks\n");
        std::printf("═══════════════════════════════════════════════════════════════\n");

        microbench_profiler_overhead();
        microbench_histogram_accuracy();
        microbench_imu_sample_throughput();
        microbench_snapshot_cost();
        microbench_report_generation();
    }

    // ── Full benchmark plans ────────────────────────────────────────────
    if (run_plans) {
        std::printf("\n═══════════════════════════════════════════════════════════════\n");
        std::printf("  Full Benchmark Plans\n");
        std::printf("═══════════════════════════════════════════════════════════════\n");

        std::vector<std::vector<BenchmarkResult>> all_results;

        using namespace thunderbird::odom;

        auto should_run = [&](const std::string& name) {
            return scenario_filter.empty() || scenario_filter == name;
        };

        if (should_run("drone")) {
            auto results = run_benchmark_plan(benchmark_plan_drone());
            all_results.push_back(std::move(results));
        }
        if (should_run("ground")) {
            auto results = run_benchmark_plan(benchmark_plan_ground_vehicle());
            all_results.push_back(std::move(results));
        }
        if (should_run("stress")) {
            auto results = run_benchmark_plan(benchmark_plan_stress());
            all_results.push_back(std::move(results));
        }

        // ── Summary ─────────────────────────────────────────────────────
        int plan_failures = 0;
        int plan_total    = 0;

        std::printf("\n═══════════════════════════════════════════════════════════════\n");
        std::printf("  Benchmark Summary\n");
        std::printf("═══════════════════════════════════════════════════════════════\n\n");

        for (const auto& results : all_results) {
            for (const auto& r : results) {
                ++plan_total;
                if (!r.passed) {
                    ++plan_failures;
                    ++total_failures;
                }
                std::printf("  [%s] %-30s", r.passed ? "PASS" : "FAIL",
                            r.scenario_name.c_str());
                if (!r.passed) {
                    std::printf("  %s", r.failure_reason.c_str());
                }
                std::printf("\n");
            }
        }

        std::printf("\n  %d/%d scenarios passed\n\n",
                    plan_total - plan_failures, plan_total);

        // ── JSON export ─────────────────────────────────────────────────
        if (!json_output_path.empty() && !all_results.empty()) {
            // Export the last snapshot for each plan.
            auto& prof = SlamProfiler::instance();
            auto snap = prof.snapshot();
            auto json = SlamProfiler::report_json(snap);

            std::ofstream ofs(json_output_path);
            if (ofs.is_open()) {
                ofs << json;
                std::printf("  JSON report written to: %s\n", json_output_path.c_str());
            } else {
                std::printf("  [WARN] Could not write JSON to: %s\n",
                            json_output_path.c_str());
            }
        }

        // ── Print last text report ──────────────────────────────────────
        {
            auto& prof = SlamProfiler::instance();
            auto snap = prof.snapshot();
            std::printf("%s", SlamProfiler::report_text(snap).c_str());
        }
    }

    std::printf("\n  Total failures: %d\n", total_failures);
    return total_failures == 0 ? 0 : 1;
}
