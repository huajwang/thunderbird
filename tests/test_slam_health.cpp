// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Health Monitor Tests
// ─────────────────────────────────────────────────────────────────────────────
//
// Standalone test harness for SlamHealthMonitor failure detection.
// Exercises every detector, state machine transition, confidence/validity
// computation, and the reporting utilities.
//
// Build:
//   cmake -DTHUNDERBIRD_BUILD_TESTS=ON ..
//   make test_slam_health
//
// Run:
//   ./test_slam_health
//
// ─────────────────────────────────────────────────────────────────────────────

#include "thunderbird/odom/slam_health.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace thunderbird::odom;

// ═════════════════════════════════════════════════════════════════════════════
//  Test infrastructure
// ═════════════════════════════════════════════════════════════════════════════

static int g_tests_run    = 0;
static int g_tests_passed = 0;

#define TEST_CASE(name)                                                   \
    do {                                                                  \
        ++g_tests_run;                                                    \
        std::printf("  %-52s ", name);                                    \
    } while (0)

#define PASS()                                                            \
    do {                                                                  \
        ++g_tests_passed;                                                 \
        std::printf("[PASS]\n");                                          \
    } while (0)

#define FAIL(msg)                                                         \
    do {                                                                  \
        std::printf("[FAIL] %s (line %d)\n", msg, __LINE__);             \
        return;                                                           \
    } while (0)

#define ASSERT_EQ(a, b)                                                   \
    do { if ((a) != (b)) FAIL(#a " != " #b); } while (0)

#define ASSERT_NE(a, b)                                                   \
    do { if ((a) == (b)) FAIL(#a " == " #b); } while (0)

#define ASSERT_TRUE(expr)                                                 \
    do { if (!(expr)) FAIL(#expr " is false"); } while (0)

#define ASSERT_FALSE(expr)                                                \
    do { if ((expr)) FAIL(#expr " is true"); } while (0)

#define ASSERT_NEAR(a, b, tol)                                            \
    do {                                                                  \
        if (std::abs(static_cast<double>(a) - static_cast<double>(b))     \
            > (tol))                                                      \
            FAIL(#a " not near " #b);                                     \
    } while (0)

// ── Helper factories ────────────────────────────────────────────────────

/// Make a nominal IMU sample at the given timestamp.
ImuSample make_imu(int64_t ts_ns,
                   double ax = 0.0, double ay = 0.0, double az = 9.81,
                   double gx = 0.0, double gy = 0.0, double gz = 0.0) {
    ImuSample s{};
    s.timestamp_ns = ts_ns;
    s.accel = {ax, ay, az};
    s.gyro  = {gx, gy, gz};
    return s;
}

/// Make a nominal SlamOutput at the given timestamp with controllable fields.
SlamOutput make_output(int64_t ts_ns,
                       double px = 0.0, double py = 0.0, double pz = 0.0,
                       double residual = 0.01,
                       uint32_t corrs = 500,
                       double cov_diag = 0.01,
                       size_t map_pts = 10000) {
    SlamOutput out{};
    out.timestamp_ns    = ts_ns;
    out.pose.timestamp_ns = ts_ns;
    out.pose.position   = {px, py, pz};
    out.esikf_residual  = residual;
    out.correspondences = corrs;
    out.map_info.total_points = map_pts;
    // Fill diagonal of 6×6 covariance.
    for (int i = 0; i < 6; ++i)
        out.pose.covariance_6x6[static_cast<size_t>(i * 6 + i)] = cov_diag;
    return out;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Enum / bitfield tests
// ═════════════════════════════════════════════════════════════════════════════

void test_health_state_names() {
    TEST_CASE("HealthState names");
    ASSERT_EQ(std::string(health_state_name(HealthState::Nominal)),   "Nominal");
    ASSERT_EQ(std::string(health_state_name(HealthState::Degraded)),  "Degraded");
    ASSERT_EQ(std::string(health_state_name(HealthState::Critical)),  "Critical");
    ASSERT_EQ(std::string(health_state_name(HealthState::Emergency)), "Emergency");
    PASS();
}

void test_output_validity_ordering() {
    TEST_CASE("OutputValidity ordering (>= for branch)");
    ASSERT_TRUE(OutputValidity::Valid      >= OutputValidity::Cautionary);
    ASSERT_TRUE(OutputValidity::Cautionary >= OutputValidity::Invalid);
    ASSERT_TRUE(OutputValidity::Invalid    >= OutputValidity::Stale);
    ASSERT_FALSE(OutputValidity::Invalid   >= OutputValidity::Cautionary);
    PASS();
}

void test_fault_flags_bitwise() {
    TEST_CASE("FaultFlags bitwise operations");
    FaultFlags f = FaultFlags::None;
    ASSERT_EQ(fault_count(f), 0);
    ASSERT_FALSE(has_fault(f, FaultFlags::ImuDropout));

    f |= FaultFlags::ImuDropout;
    f |= FaultFlags::EkfDivergence;
    ASSERT_EQ(fault_count(f), 2);
    ASSERT_TRUE(has_fault(f, FaultFlags::ImuDropout));
    ASSERT_TRUE(has_fault(f, FaultFlags::EkfDivergence));
    ASSERT_FALSE(has_fault(f, FaultFlags::LidarDropout));

    // AND test.
    FaultFlags masked = f & FaultFlags::ImuDropout;
    ASSERT_TRUE(has_fault(masked, FaultFlags::ImuDropout));
    ASSERT_FALSE(has_fault(masked, FaultFlags::EkfDivergence));
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════════
//  Initial state tests
// ═════════════════════════════════════════════════════════════════════════════

void test_initial_state() {
    TEST_CASE("SlamHealthMonitor initial state");
    SlamHealthMonitor mon;
    ASSERT_EQ(mon.state(), HealthState::Nominal);
    ASSERT_NEAR(mon.confidence(), 1.0f, 0.001f);
    ASSERT_EQ(mon.validity(), OutputValidity::Valid);
    ASSERT_EQ(mon.faults(), FaultFlags::None);

    auto snap = mon.snapshot();
    ASSERT_EQ(snap.state, HealthState::Nominal);
    ASSERT_EQ(snap.active_faults, FaultFlags::None);
    ASSERT_EQ(snap.total_imu_dropouts, 0u);
    ASSERT_EQ(snap.total_lidar_dropouts, 0u);
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════════
//  IMU dropout detection
// ═════════════════════════════════════════════════════════════════════════════

void test_imu_nominal_rate() {
    TEST_CASE("IMU nominal rate — no dropout");
    HealthMonitorConfig cfg;
    cfg.imu_rate_hz = 400.0;
    SlamHealthMonitor mon(cfg);

    // Feed IMU at 400 Hz (2.5 ms period) for 50 samples.
    const int64_t period_ns = 2'500'000; // 2.5 ms
    for (int i = 0; i < 50; ++i) {
        mon.on_imu(make_imu(period_ns * (i + 1)));
    }
    auto snap = mon.snapshot();
    ASSERT_FALSE(has_fault(snap.active_faults, FaultFlags::ImuDropout));
    ASSERT_NEAR(snap.imu_score, 1.0f, 0.01f);
    PASS();
}

void test_imu_dropout_detected() {
    TEST_CASE("IMU dropout — gap > 3× period");
    HealthMonitorConfig cfg;
    cfg.imu_rate_hz = 400.0;
    cfg.dropout_factor = 3.0;
    SlamHealthMonitor mon(cfg);

    const int64_t period_ns = 2'500'000;
    // Feed 10 normal samples.
    for (int i = 0; i < 10; ++i) {
        mon.on_imu(make_imu(period_ns * (i + 1)));
    }
    // Then a gap of 10× period.
    int64_t t_gap = period_ns * 10 + period_ns * 10; // 10 periods after last
    mon.on_imu(make_imu(t_gap));

    // Also feed a SlamOutput to trigger state machine.
    mon.on_slam_output(make_output(t_gap));

    auto snap = mon.snapshot();
    ASSERT_TRUE(has_fault(snap.active_faults, FaultFlags::ImuDropout));
    ASSERT_TRUE(snap.imu_score < 0.5f);
    ASSERT_TRUE(snap.total_imu_dropouts > 0u);
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════════
//  LiDAR dropout detection
// ═════════════════════════════════════════════════════════════════════════════

void test_lidar_dropout_detected() {
    TEST_CASE("LiDAR dropout — gap > 3× scan period");
    HealthMonitorConfig cfg;
    cfg.lidar_rate_hz = 10.0;
    cfg.dropout_factor = 3.0;
    SlamHealthMonitor mon(cfg);

    const int64_t scan_period_ns = 100'000'000; // 100 ms
    // Feed 5 normal scans.
    for (int i = 0; i < 5; ++i) {
        mon.on_lidar(scan_period_ns * (i + 1));
    }
    // Gap of 5× scan period.
    int64_t t_gap = scan_period_ns * 5 + scan_period_ns * 5;
    mon.on_lidar(t_gap);

    // Trigger state machine via a SlamOutput.
    mon.on_slam_output(make_output(t_gap));

    auto snap = mon.snapshot();
    ASSERT_TRUE(has_fault(snap.active_faults, FaultFlags::LidarDropout));
    ASSERT_TRUE(snap.lidar_score < 0.5f);
    ASSERT_TRUE(snap.total_lidar_dropouts > 0u);
    PASS();
}

void test_lidar_nominal_rate() {
    TEST_CASE("LiDAR nominal rate — no dropout");
    HealthMonitorConfig cfg;
    cfg.lidar_rate_hz = 10.0;
    SlamHealthMonitor mon(cfg);

    const int64_t scan_period_ns = 100'000'000;
    for (int i = 0; i < 20; ++i) {
        mon.on_lidar(scan_period_ns * (i + 1));
    }
    auto snap = mon.snapshot();
    ASSERT_FALSE(has_fault(snap.active_faults, FaultFlags::LidarDropout));
    ASSERT_NEAR(snap.lidar_score, 1.0f, 0.01f);
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════════
//  EKF divergence (covariance trace)
// ═════════════════════════════════════════════════════════════════════════════

void test_ekf_nominal_covariance() {
    TEST_CASE("EKF nominal covariance (trace < warn)");
    SlamHealthMonitor mon;

    // Feed IMU + LiDAR first so dropout detectors don't fire.
    mon.on_imu(make_imu(1'000'000));
    mon.on_lidar(1'000'000);
    mon.on_slam_output(make_output(1'000'000, 0, 0, 0, 0.01, 500, 0.01));

    auto snap = mon.snapshot();
    ASSERT_FALSE(has_fault(snap.active_faults, FaultFlags::EkfDivergence));
    ASSERT_NEAR(snap.ekf_score, 1.0f, 0.01f);
    PASS();
}

void test_ekf_divergence_high_trace() {
    TEST_CASE("EKF divergence — high covariance trace");
    HealthMonitorConfig cfg;
    cfg.cov_trace_warn = 10.0;
    cfg.cov_trace_crit = 100.0;
    SlamHealthMonitor mon(cfg);

    // cov_diag=20 → trace=120 > crit threshold
    mon.on_imu(make_imu(1'000'000));
    mon.on_lidar(1'000'000);
    mon.on_slam_output(make_output(1'000'000, 0, 0, 0, 0.01, 500, 20.0));

    auto snap = mon.snapshot();
    ASSERT_TRUE(has_fault(snap.active_faults, FaultFlags::EkfDivergence));
    ASSERT_NEAR(snap.ekf_score, 0.0f, 0.01f);
    PASS();
}

void test_ekf_divergence_nan() {
    TEST_CASE("EKF divergence — NaN in covariance");
    SlamHealthMonitor mon;

    SlamOutput out = make_output(1'000'000);
    out.pose.covariance_6x6[0] = std::numeric_limits<double>::quiet_NaN();
    mon.on_imu(make_imu(1'000'000));
    mon.on_lidar(1'000'000);
    mon.on_slam_output(out);

    auto snap = mon.snapshot();
    ASSERT_TRUE(has_fault(snap.active_faults, FaultFlags::CovarianceNaN));
    ASSERT_TRUE(has_fault(snap.active_faults, FaultFlags::EkfDivergence));
    ASSERT_NEAR(snap.ekf_score, 0.0f, 0.01f);
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════════
//  Residual spike detection
// ═════════════════════════════════════════════════════════════════════════════

void test_residual_spike() {
    TEST_CASE("Residual spike — 10× higher than baseline");
    HealthMonitorConfig cfg;
    cfg.residual_spike_factor = 3.0;
    cfg.residual_ema_alpha    = 0.1;
    SlamHealthMonitor mon(cfg);

    const int64_t period_ns = 100'000'000;

    // Build up a baseline EMA with low residuals.
    for (int i = 0; i < 30; ++i) {
        mon.on_imu(make_imu(period_ns * (i + 1)));
        mon.on_lidar(period_ns * (i + 1));
        mon.on_slam_output(make_output(period_ns * (i + 1),
            0, 0, 0,  /*residual=*/0.01));
    }

    // Now inject a massive spike.
    int64_t t_spike = period_ns * 31;
    mon.on_imu(make_imu(t_spike));
    mon.on_lidar(t_spike);
    mon.on_slam_output(make_output(t_spike, 0, 0, 0, /*residual=*/1.0));

    auto snap = mon.snapshot();
    ASSERT_TRUE(has_fault(snap.active_faults, FaultFlags::ResidualSpike));
    ASSERT_TRUE(snap.residual_score < 1.0f);
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════════
//  High drift rate detection
// ═════════════════════════════════════════════════════════════════════════════

void test_drift_rate_normal() {
    TEST_CASE("Drift rate normal — slow motion");
    HealthMonitorConfig cfg;
    cfg.max_platform_speed_mps = 30.0;
    cfg.drift_speed_factor     = 2.0;
    SlamHealthMonitor mon(cfg);

    const int64_t scan_ns = 100'000'000; // 100 ms = 10 Hz
    // Move 0.1 m per scan → 1 m/s.  Well within 30 × 2 = 60 m/s limit.
    for (int i = 0; i < 10; ++i) {
        int64_t t = scan_ns * (i + 1);
        mon.on_imu(make_imu(t));
        mon.on_lidar(t);
        mon.on_slam_output(make_output(t,
            /*px=*/0.1 * (i + 1), 0, 0));
    }

    auto snap = mon.snapshot();
    ASSERT_FALSE(has_fault(snap.active_faults, FaultFlags::HighDriftRate));
    ASSERT_NEAR(snap.drift_score, 1.0f, 0.01f);
    PASS();
}

void test_drift_rate_excessive() {
    TEST_CASE("Drift rate excessive — impossible teleport");
    HealthMonitorConfig cfg;
    cfg.max_platform_speed_mps = 30.0;
    cfg.drift_speed_factor     = 2.0;
    SlamHealthMonitor mon(cfg);

    const int64_t scan_ns = 100'000'000; // 100 ms

    // First output at origin.
    mon.on_imu(make_imu(scan_ns));
    mon.on_lidar(scan_ns);
    mon.on_slam_output(make_output(scan_ns, 0, 0, 0));

    // Second output: jumped 100 m in 100 ms → 1000 m/s.
    mon.on_imu(make_imu(scan_ns * 2));
    mon.on_lidar(scan_ns * 2);
    mon.on_slam_output(make_output(scan_ns * 2, 100.0, 0, 0));

    auto snap = mon.snapshot();
    ASSERT_TRUE(has_fault(snap.active_faults, FaultFlags::HighDriftRate));
    ASSERT_TRUE(snap.drift_score < 0.5f);
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════════
//  Degenerate geometry (eigenvalue ratio)
// ═════════════════════════════════════════════════════════════════════════════

void test_geometry_good_3d() {
    TEST_CASE("Geometry — good 3D structure");
    HealthMonitorConfig cfg;
    cfg.geom_degen_ratio_warn = 0.01;
    cfg.min_points_for_geom = 50;
    SlamHealthMonitor mon(cfg);

    // Set a healthy eigenvalue ratio (close to 1 = isotropic).
    mon.set_eigenvalue_ratio(0.5);

    mon.on_imu(make_imu(1'000'000));
    mon.on_lidar(1'000'000);
    mon.on_slam_output(make_output(1'000'000, 0, 0, 0, 0.01, 500));

    auto snap = mon.snapshot();
    ASSERT_FALSE(has_fault(snap.active_faults, FaultFlags::DegenerateGeom));
    ASSERT_NEAR(snap.geometry_score, 1.0f, 0.01f);
    PASS();
}

void test_geometry_degenerate() {
    TEST_CASE("Geometry — degenerate (tunnel-like)");
    HealthMonitorConfig cfg;
    cfg.geom_degen_ratio_warn = 0.01;
    cfg.geom_linear_ratio     = 0.001;
    cfg.min_points_for_geom = 50;
    SlamHealthMonitor mon(cfg);

    // Nearly linear: ratio very small.
    mon.set_eigenvalue_ratio(0.0005);

    mon.on_imu(make_imu(1'000'000));
    mon.on_lidar(1'000'000);
    mon.on_slam_output(make_output(1'000'000, 0, 0, 0, 0.01, 500));

    auto snap = mon.snapshot();
    ASSERT_TRUE(has_fault(snap.active_faults, FaultFlags::DegenerateGeom));
    ASSERT_NEAR(snap.geometry_score, 0.0f, 0.01f);
    PASS();
}

void test_eigenvalue_ratio_3x3_identity() {
    TEST_CASE("eigenvalue_ratio_3x3 — identity matrix");
    // Identity (upper tri): [1, 0, 0, 1, 0, 1]
    const double scatter[6] = {1, 0, 0, 1, 0, 1};
    double ratio = eigenvalue_ratio_3x3(scatter);
    ASSERT_NEAR(ratio, 1.0, 0.01);
    PASS();
}

void test_eigenvalue_ratio_3x3_planar() {
    TEST_CASE("eigenvalue_ratio_3x3 — planar (one small eigenvalue)");
    // Scatter dominated by two axes: λ = {500, 300, 0.01}
    // (Non-repeated eigenvalues avoid trigonometric solver degeneracy.)
    const double scatter[6] = {500.0, 0, 0, 300.0, 0, 0.01};
    double ratio = eigenvalue_ratio_3x3(scatter);
    ASSERT_TRUE(ratio < 0.01);
    ASSERT_TRUE(ratio > 0.0);
    PASS();
}

void test_eigenvalue_ratio_3x3_linear() {
    TEST_CASE("eigenvalue_ratio_3x3 — linear (two small eigenvalues)");
    // One dominant axis: λ = {1000, 0.01, 0.01}
    const double scatter[6] = {1000.0, 0, 0, 0.01, 0, 0.01};
    double ratio = eigenvalue_ratio_3x3(scatter);
    ASSERT_TRUE(ratio < 0.001);
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════════
//  IMU saturation detection
// ═════════════════════════════════════════════════════════════════════════════

void test_imu_saturation() {
    TEST_CASE("IMU saturation — accel beyond sensor range");
    HealthMonitorConfig cfg;
    cfg.accel_max_mps2 = 156.0; // ±16g
    SlamHealthMonitor mon(cfg);

    // Normal sample, then saturated.
    mon.on_imu(make_imu(1'000'000, 0, 0, 9.81));
    mon.on_imu(make_imu(2'000'000, 200.0, 0, 0));  // > 156 m/s²

    // Trigger state machine.
    mon.on_slam_output(make_output(2'000'000));

    auto snap = mon.snapshot();
    ASSERT_TRUE(has_fault(snap.active_faults, FaultFlags::ImuSaturation));
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════════
//  Map health detection
// ═════════════════════════════════════════════════════════════════════════════

void test_map_too_sparse() {
    TEST_CASE("Map too sparse — below min threshold");
    HealthMonitorConfig cfg;
    cfg.map_min_points = 100;
    SlamHealthMonitor mon(cfg);

    mon.on_imu(make_imu(1'000'000));
    mon.on_lidar(1'000'000);
    mon.on_slam_output(make_output(1'000'000, 0, 0, 0, 0.01, 500, 0.01, 10));

    auto snap = mon.snapshot();
    ASSERT_TRUE(has_fault(snap.active_faults, FaultFlags::MapDegraded));
    PASS();
}

void test_map_too_large() {
    TEST_CASE("Map too large — above max threshold");
    HealthMonitorConfig cfg;
    cfg.map_max_points = 1'000'000;
    SlamHealthMonitor mon(cfg);

    mon.on_imu(make_imu(1'000'000));
    mon.on_lidar(1'000'000);
    mon.on_slam_output(make_output(1'000'000, 0, 0, 0, 0.01, 500, 0.01,
                                   2'000'000));

    auto snap = mon.snapshot();
    ASSERT_TRUE(has_fault(snap.active_faults, FaultFlags::MapDegraded));
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════════
//  Time sync detection
// ═════════════════════════════════════════════════════════════════════════════

void test_clock_drift_within_tolerance() {
    TEST_CASE("Clock drift — within tolerance");
    HealthMonitorConfig cfg;
    cfg.time_sync_max_drift_ns = 50'000'000;
    SlamHealthMonitor mon(cfg);

    mon.set_clock_drift(10'000'000); // 10 ms — ok
    mon.on_imu(make_imu(1'000'000));
    mon.on_slam_output(make_output(1'000'000));

    auto snap = mon.snapshot();
    ASSERT_FALSE(has_fault(snap.active_faults, FaultFlags::TimeSyncLost));
    PASS();
}

void test_clock_drift_exceeded() {
    TEST_CASE("Clock drift — exceeded tolerance");
    HealthMonitorConfig cfg;
    cfg.time_sync_max_drift_ns = 50'000'000;
    SlamHealthMonitor mon(cfg);

    mon.set_clock_drift(80'000'000); // 80 ms > 50 ms limit
    mon.on_imu(make_imu(1'000'000));
    mon.on_slam_output(make_output(1'000'000));

    auto snap = mon.snapshot();
    ASSERT_TRUE(has_fault(snap.active_faults, FaultFlags::TimeSyncLost));
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════════
//  State machine transitions
// ═════════════════════════════════════════════════════════════════════════════

void test_nominal_to_degraded() {
    TEST_CASE("State: Nominal → Degraded (single fault)");
    HealthMonitorConfig cfg;
    cfg.cov_trace_warn = 10.0;
    cfg.cov_trace_crit = 100.0;
    SlamHealthMonitor mon(cfg);

    // Inject a warning-level covariance trace (cov_diag=3 → trace=18 > warn=10).
    mon.on_imu(make_imu(1'000'000));
    mon.on_lidar(1'000'000);
    mon.on_slam_output(make_output(1'000'000, 0, 0, 0, 0.01, 500, 3.0));

    ASSERT_EQ(mon.state(), HealthState::Degraded);
    PASS();
}

void test_degraded_to_critical_two_faults() {
    TEST_CASE("State: Degraded → Critical (≥2 faults)");
    HealthMonitorConfig cfg;
    cfg.cov_trace_warn = 10.0;
    cfg.cov_trace_crit = 100.0;
    cfg.accel_max_mps2 = 156.0;
    SlamHealthMonitor mon(cfg);

    // First update: Nominal → Degraded (≥1 fault fires).
    mon.on_imu(make_imu(1'000'000, 200.0, 0, 0));
    mon.on_lidar(1'000'000);
    mon.on_slam_output(make_output(1'000'000, 0, 0, 0, 0.01, 500, 20.0));
    ASSERT_EQ(mon.state(), HealthState::Degraded);

    // Second update: already Degraded + ≥2 faults → Critical.
    mon.on_imu(make_imu(2'000'000, 200.0, 0, 0));
    mon.on_lidar(2'000'000);
    mon.on_slam_output(make_output(2'000'000, 0, 0, 0, 0.01, 500, 20.0));
    ASSERT_EQ(mon.state(), HealthState::Critical);
    PASS();
}

void test_degraded_to_critical_timeout() {
    TEST_CASE("State: Degraded → Critical (timeout)");
    HealthMonitorConfig cfg;
    cfg.cov_trace_warn = 10.0;
    cfg.cov_trace_crit = 100.0;
    cfg.degrade_timeout_ns = 500'000'000; // 0.5 s for fast test
    SlamHealthMonitor mon(cfg);

    // Single fault persisting over time.
    const int64_t step_ns = 100'000'000; // 100 ms
    for (int i = 0; i < 10; ++i) {
        int64_t t = step_ns * (i + 1);
        mon.on_imu(make_imu(t));
        mon.on_lidar(t);
        // cov_diag=3 → trace=18 > warn=10, but < crit=100 → single fault.
        mon.on_slam_output(make_output(t, 0, 0, 0, 0.01, 500, 3.0));
    }
    // After 1s with a 0.5s timeout, should be Critical.
    ASSERT_EQ(mon.state(), HealthState::Critical);
    PASS();
}

void test_critical_to_emergency_timeout() {
    TEST_CASE("State: Critical → Emergency (timeout)");
    HealthMonitorConfig cfg;
    cfg.cov_trace_warn = 10.0;
    cfg.cov_trace_crit = 100.0;
    cfg.accel_max_mps2 = 156.0;
    cfg.degrade_timeout_ns  = 100'000'000;  // 100 ms
    cfg.critical_timeout_ns = 200'000'000;  // 200 ms
    SlamHealthMonitor mon(cfg);

    const int64_t step_ns = 100'000'000; // 100 ms
    for (int i = 0; i < 10; ++i) {
        int64_t t = step_ns * (i + 1);
        // Two persistent faults: saturation + EKF divergence.
        mon.on_imu(make_imu(t, 200.0, 0, 0));
        mon.on_lidar(t);
        mon.on_slam_output(make_output(t, 0, 0, 0, 0.01, 500, 20.0));
    }
    ASSERT_EQ(mon.state(), HealthState::Emergency);
    // Emergency forces validity to Invalid.
    ASSERT_EQ(mon.validity(), OutputValidity::Invalid);
    PASS();
}

void test_heal_from_degraded() {
    TEST_CASE("State: Degraded → Nominal (heal)");
    HealthMonitorConfig cfg;
    cfg.cov_trace_warn = 10.0;
    cfg.cov_trace_crit = 100.0;
    cfg.heal_window_ns = 200'000'000; // 200 ms for fast test
    // Use 10 Hz sensor rates so 100 ms steps are on-time.
    cfg.imu_rate_hz   = 10.0;
    cfg.lidar_rate_hz = 10.0;
    cfg.dropout_factor = 3.0;
    SlamHealthMonitor mon(cfg);

    // First: cause a fault → Degraded.
    mon.on_imu(make_imu(100'000'000));
    mon.on_lidar(100'000'000);
    mon.on_slam_output(make_output(100'000'000, 0, 0, 0, 0.01, 500, 3.0));
    ASSERT_EQ(mon.state(), HealthState::Degraded);

    // Now: feed clean data at 100 ms intervals (within 10 Hz tolerance).
    const int64_t step_ns = 100'000'000;
    for (int i = 0; i < 10; ++i) {
        int64_t t = 100'000'000 + step_ns * (i + 1);
        mon.on_imu(make_imu(t));
        mon.on_lidar(t);
        mon.on_slam_output(make_output(t, 0, 0, 0, 0.01, 500, 0.01));
    }
    ASSERT_EQ(mon.state(), HealthState::Nominal);
    PASS();
}

void test_emergency_requires_double_heal() {
    TEST_CASE("Emergency recovery requires 2× heal window");
    HealthMonitorConfig cfg;
    cfg.cov_trace_warn = 10.0;
    cfg.cov_trace_crit = 100.0;
    cfg.accel_max_mps2 = 156.0;
    cfg.degrade_timeout_ns  = 50'000'000;   // 50 ms
    cfg.critical_timeout_ns = 100'000'000;  // 100 ms
    cfg.heal_window_ns = 200'000'000;        // 200 ms
    // Use 20 Hz sensor rates so 50 ms steps are on-time.
    cfg.imu_rate_hz   = 20.0;
    cfg.lidar_rate_hz = 20.0;
    cfg.dropout_factor = 3.0;
    SlamHealthMonitor mon(cfg);

    const int64_t step_ns = 50'000'000; // 50 ms

    // Drive into Emergency.
    for (int i = 0; i < 20; ++i) {
        int64_t t = step_ns * (i + 1);
        mon.on_imu(make_imu(t, 200.0, 0, 0));
        mon.on_lidar(t);
        mon.on_slam_output(make_output(t, 0, 0, 0, 0.01, 500, 20.0));
    }
    ASSERT_EQ(mon.state(), HealthState::Emergency);

    // Clear faults but not long enough (only 1× heal_window).
    for (int i = 0; i < 4; ++i) { // 4 × 50ms = 200ms = 1× heal
        int64_t t = step_ns * (20 + i + 1);
        mon.on_imu(make_imu(t));
        mon.on_lidar(t);
        mon.on_slam_output(make_output(t, 0, 0, 0, 0.01, 500, 0.01));
    }
    // Should still be Emergency — needs 2× heal_window = 400 ms.
    ASSERT_EQ(mon.state(), HealthState::Emergency);

    // Continue clear for another 1× heal_window.
    for (int i = 0; i < 6; ++i) { // 6 × 50ms = 300ms extra
        int64_t t = step_ns * (24 + i + 1);
        mon.on_imu(make_imu(t));
        mon.on_lidar(t);
        mon.on_slam_output(make_output(t, 0, 0, 0, 0.01, 500, 0.01));
    }
    // Should have healed to Degraded by now (Emergency → Degraded).
    ASSERT_NE(mon.state(), HealthState::Emergency);
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════════
//  Confidence & validity computation
// ═════════════════════════════════════════════════════════════════════════════

void test_confidence_all_nominal() {
    TEST_CASE("Confidence 1.0 when all nominal");
    SlamHealthMonitor mon;

    mon.on_imu(make_imu(1'000'000));
    mon.on_lidar(1'000'000);
    mon.on_slam_output(make_output(1'000'000, 0, 0, 0, 0.01, 500, 0.01));

    ASSERT_NEAR(mon.confidence(), 1.0f, 0.05f);
    ASSERT_EQ(mon.validity(), OutputValidity::Valid);
    PASS();
}

void test_confidence_drops_with_faults() {
    TEST_CASE("Confidence drops when detectors fire");
    HealthMonitorConfig cfg;
    cfg.cov_trace_warn = 10.0;
    cfg.cov_trace_crit = 100.0;
    SlamHealthMonitor mon(cfg);

    // EKF diverged: cov_diag=20 → trace=120 → ekf_score=0.0.
    mon.on_imu(make_imu(1'000'000));
    mon.on_lidar(1'000'000);
    mon.on_slam_output(make_output(1'000'000, 0, 0, 0, 0.01, 500, 20.0));

    // Confidence should be < 1.0 (ekf weight = 0.20, score = 0.0).
    // Expected: 0.20*1 + 0.20*1 + 0.15*1 + 0.20*0 + 0.15*1 + 0.10*1 = 0.80
    ASSERT_NEAR(mon.confidence(), 0.80f, 0.05f);
    PASS();
}

void test_validity_cautionary() {
    TEST_CASE("Validity → Cautionary when confidence ~0.5");
    HealthMonitorConfig cfg;
    cfg.cov_trace_warn = 10.0;
    cfg.cov_trace_crit = 100.0;
    cfg.validity_cautionary_threshold = 0.8;
    cfg.validity_invalid_threshold = 0.3;
    SlamHealthMonitor mon(cfg);

    // Multiple degraded detectors to bring confidence to ~0.5.
    // Large IMU gap: score drops.
    const int64_t imu_period_ns = 2'500'000;
    mon.on_imu(make_imu(imu_period_ns));
    mon.on_imu(make_imu(imu_period_ns * 10)); // 10× period → dropout, low score

    // High covariance.
    mon.on_lidar(imu_period_ns * 10);
    mon.on_slam_output(make_output(imu_period_ns * 10, 0, 0, 0, 0.01, 500, 10.0));

    float conf = mon.confidence();
    // With IMU score low and EKF score around 0.4-0.5, confidence should be
    // below 0.8 → Cautionary.
    if (conf >= cfg.validity_invalid_threshold && conf < cfg.validity_cautionary_threshold) {
        ASSERT_EQ(mon.validity(), OutputValidity::Cautionary);
    }
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════════
//  Callbacks
// ═════════════════════════════════════════════════════════════════════════════

void test_state_change_callback() {
    TEST_CASE("State change callback fires on transition");
    HealthMonitorConfig cfg;
    cfg.cov_trace_warn = 10.0;
    cfg.cov_trace_crit = 100.0;
    SlamHealthMonitor mon(cfg);

    int callback_count = 0;
    HealthState last_from = HealthState::Nominal;
    HealthState last_to   = HealthState::Nominal;

    mon.on_health_state_change([&](HealthState from, HealthState to, FaultFlags) {
        ++callback_count;
        last_from = from;
        last_to   = to;
    });

    // Trigger Nominal → Degraded.
    mon.on_imu(make_imu(1'000'000));
    mon.on_lidar(1'000'000);
    mon.on_slam_output(make_output(1'000'000, 0, 0, 0, 0.01, 500, 3.0));

    ASSERT_EQ(callback_count, 1);
    ASSERT_EQ(last_from, HealthState::Nominal);
    ASSERT_EQ(last_to, HealthState::Degraded);
    PASS();
}

void test_emergency_callback() {
    TEST_CASE("Emergency callback fires on Emergency entry");
    HealthMonitorConfig cfg;
    cfg.cov_trace_warn = 10.0;
    cfg.cov_trace_crit = 100.0;
    cfg.accel_max_mps2 = 156.0;
    cfg.degrade_timeout_ns  = 50'000'000;
    cfg.critical_timeout_ns = 100'000'000;
    SlamHealthMonitor mon(cfg);

    int emergency_count = 0;
    mon.on_emergency([&](const SlamHealthSnapshot&) {
        ++emergency_count;
    });

    const int64_t step_ns = 50'000'000;
    for (int i = 0; i < 20; ++i) {
        int64_t t = step_ns * (i + 1);
        mon.on_imu(make_imu(t, 200.0, 0, 0));
        mon.on_lidar(t);
        mon.on_slam_output(make_output(t, 0, 0, 0, 0.01, 500, 20.0));
    }

    ASSERT_TRUE(emergency_count >= 1);
    PASS();
}

void test_health_update_callback() {
    TEST_CASE("Health update callback fires on every on_slam_output");
    SlamHealthMonitor mon;

    int update_count = 0;
    mon.on_health_update([&](const SlamHealthSnapshot&) {
        ++update_count;
    });

    for (int i = 0; i < 5; ++i) {
        int64_t t = 100'000'000 * (i + 1);
        mon.on_imu(make_imu(t));
        mon.on_lidar(t);
        mon.on_slam_output(make_output(t));
    }

    ASSERT_EQ(update_count, 5);
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════════
//  Reset
// ═════════════════════════════════════════════════════════════════════════════

void test_reset() {
    TEST_CASE("Reset clears all state");
    HealthMonitorConfig cfg;
    cfg.cov_trace_warn = 10.0;
    cfg.cov_trace_crit = 100.0;
    cfg.accel_max_mps2 = 156.0;
    cfg.degrade_timeout_ns  = 50'000'000;
    cfg.critical_timeout_ns = 100'000'000;
    SlamHealthMonitor mon(cfg);

    // Drive into bad state.
    const int64_t step_ns = 50'000'000;
    for (int i = 0; i < 20; ++i) {
        int64_t t = step_ns * (i + 1);
        mon.on_imu(make_imu(t, 200.0, 0, 0));
        mon.on_lidar(t);
        mon.on_slam_output(make_output(t, 0, 0, 0, 0.01, 500, 20.0));
    }
    ASSERT_NE(mon.state(), HealthState::Nominal);

    mon.reset();

    ASSERT_EQ(mon.state(), HealthState::Nominal);
    ASSERT_NEAR(mon.confidence(), 1.0f, 0.001f);
    ASSERT_EQ(mon.validity(), OutputValidity::Valid);
    ASSERT_EQ(mon.faults(), FaultFlags::None);

    auto snap = mon.snapshot();
    ASSERT_EQ(snap.total_imu_dropouts, 0u);
    ASSERT_EQ(snap.total_state_transitions, 0u);
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════════
//  Config hot-reload
// ═════════════════════════════════════════════════════════════════════════════

void test_config_update() {
    TEST_CASE("Config hot-reload changes thresholds");
    HealthMonitorConfig cfg;
    cfg.imu_rate_hz = 400.0;
    SlamHealthMonitor mon(cfg);

    ASSERT_NEAR(mon.config().imu_rate_hz, 400.0, 0.1);

    cfg.imu_rate_hz = 200.0;
    mon.update_config(cfg);

    ASSERT_NEAR(mon.config().imu_rate_hz, 200.0, 0.1);
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════════
//  Snapshot completeness
// ═════════════════════════════════════════════════════════════════════════════

void test_snapshot_completeness() {
    TEST_CASE("Snapshot captures all diagnostic fields");
    SlamHealthMonitor mon;

    const int64_t t = 500'000'000;
    mon.on_imu(make_imu(t, 0, 0, 9.81));
    mon.on_lidar(t);
    mon.set_eigenvalue_ratio(0.7);
    mon.set_clock_drift(5'000'000);
    mon.on_slam_output(make_output(t, 1.0, 2.0, 3.0, 0.005, 800, 0.02, 5000));

    auto snap = mon.snapshot();
    ASSERT_EQ(snap.timestamp_ns, t);
    ASSERT_TRUE(snap.last_imu_ns > 0);
    ASSERT_TRUE(snap.last_lidar_ns > 0);
    ASSERT_NEAR(snap.eigenvalue_ratio, 0.7, 0.01);
    ASSERT_TRUE(snap.cov_trace > 0.0);
    ASSERT_TRUE(snap.residual_ema >= 0.0);
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════════
//  Reporting utilities (from slam_health.cpp)
// ═════════════════════════════════════════════════════════════════════════════

void test_fault_flags_to_string() {
    TEST_CASE("fault_flags_to_string");
    ASSERT_EQ(fault_flags_to_string(FaultFlags::None), std::string("None"));

    auto s = fault_flags_to_string(FaultFlags::ImuDropout | FaultFlags::EkfDivergence);
    ASSERT_TRUE(s.find("ImuDropout") != std::string::npos);
    ASSERT_TRUE(s.find("EkfDivergence") != std::string::npos);
    PASS();
}

void test_snapshot_json() {
    TEST_CASE("health_snapshot_to_json produces valid output");
    SlamHealthMonitor mon;
    mon.on_imu(make_imu(1'000'000));
    mon.on_lidar(1'000'000);
    mon.on_slam_output(make_output(1'000'000));

    std::string json = health_snapshot_to_json(mon.snapshot());
    ASSERT_TRUE(json.find("\"state\":") != std::string::npos);
    ASSERT_TRUE(json.find("\"confidence\":") != std::string::npos);
    ASSERT_TRUE(json.find("\"scores\":") != std::string::npos);
    ASSERT_TRUE(json.find("Nominal") != std::string::npos);
    PASS();
}

void test_snapshot_text() {
    TEST_CASE("health_snapshot_to_text produces readable output");
    SlamHealthMonitor mon;
    mon.on_imu(make_imu(1'000'000));
    mon.on_lidar(1'000'000);
    mon.on_slam_output(make_output(1'000'000));

    std::string text = health_snapshot_to_text(mon.snapshot());
    ASSERT_TRUE(text.find("SLAM Health:") != std::string::npos);
    ASSERT_TRUE(text.find("confidence=") != std::string::npos);
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════════
//  Main
// ═════════════════════════════════════════════════════════════════════════════

int main() {
    std::printf("═══════════════════════════════════════════════════════════\n");
    std::printf("  Thunderbird SDK — SLAM Health Monitor Tests\n");
    std::printf("═══════════════════════════════════════════════════════════\n\n");

    // Enum / bitfield.
    test_health_state_names();
    test_output_validity_ordering();
    test_fault_flags_bitwise();

    // Initial state.
    test_initial_state();

    // IMU dropout.
    test_imu_nominal_rate();
    test_imu_dropout_detected();

    // LiDAR dropout.
    test_lidar_dropout_detected();
    test_lidar_nominal_rate();

    // EKF divergence.
    test_ekf_nominal_covariance();
    test_ekf_divergence_high_trace();
    test_ekf_divergence_nan();

    // Residual spike.
    test_residual_spike();

    // Drift rate.
    test_drift_rate_normal();
    test_drift_rate_excessive();

    // Degenerate geometry.
    test_geometry_good_3d();
    test_geometry_degenerate();
    test_eigenvalue_ratio_3x3_identity();
    test_eigenvalue_ratio_3x3_planar();
    test_eigenvalue_ratio_3x3_linear();

    // IMU saturation.
    test_imu_saturation();

    // Map health.
    test_map_too_sparse();
    test_map_too_large();

    // Time sync.
    test_clock_drift_within_tolerance();
    test_clock_drift_exceeded();

    // State machine.
    test_nominal_to_degraded();
    test_degraded_to_critical_two_faults();
    test_degraded_to_critical_timeout();
    test_critical_to_emergency_timeout();
    test_heal_from_degraded();
    test_emergency_requires_double_heal();

    // Confidence & validity.
    test_confidence_all_nominal();
    test_confidence_drops_with_faults();
    test_validity_cautionary();

    // Callbacks.
    test_state_change_callback();
    test_emergency_callback();
    test_health_update_callback();

    // Reset.
    test_reset();

    // Config.
    test_config_update();

    // Snapshot.
    test_snapshot_completeness();

    // Reporting utilities.
    test_fault_flags_to_string();
    test_snapshot_json();
    test_snapshot_text();

    std::printf("\n═══════════════════════════════════════════════════════════\n");
    std::printf("  Results: %d / %d passed\n", g_tests_passed, g_tests_run);
    std::printf("═══════════════════════════════════════════════════════════\n");

    return (g_tests_passed == g_tests_run) ? 0 : 1;
}
