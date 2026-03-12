// ─────────────────────────────────────────────────────────────────────────────
// Test — IMU B-Spline Interpolation (Phase 8)
// ─────────────────────────────────────────────────────────────────────────────
//
// Tests the ImuInterpolator::bspline() method and verifies the B-spline
// boundary interpolation upgrade in SlamTimeSync can be enabled via config.
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/odom/slam_time_sync.h"

#include <cassert>
#include <cmath>
#include <numbers>
#include <cstdio>
#include <vector>

using namespace thunderbird::odom;

static constexpr double kEps = 1e-4;

static bool near(double a, double b, double tol = kEps) {
    return std::abs(a - b) < tol;
}

static ImuSample make_imu(int64_t ts, double ax, double ay, double az,
                           double gx = 0, double gy = 0, double gz = 0) {
    ImuSample s;
    s.timestamp_ns = ts;
    s.accel = {ax, ay, az};
    s.gyro  = {gx, gy, gz};
    s.temperature = 25.0;
    return s;
}

// ── lerp: sanity baseline ───────────────────────────────────────────────────

static void test_lerp_midpoint() {
    auto a = make_imu(1000, 0.0, 0.0, 9.81);
    auto b = make_imu(2000, 2.0, 0.0, 9.81);

    auto mid = ImuInterpolator::lerp(a, b, 1500);
    assert(mid.timestamp_ns == 1500);
    assert(near(mid.accel[0], 1.0));
    assert(near(mid.accel[2], 9.81));
    std::puts("  lerp midpoint            OK");
}

static void test_lerp_clamped_outside() {
    auto a = make_imu(1000, 0.0, 0.0, 9.81);
    auto b = make_imu(2000, 2.0, 0.0, 9.81);

    auto before = ImuInterpolator::lerp_clamped(a, b, 500);
    assert(before.timestamp_ns == a.timestamp_ns);  // clamped to a

    auto after = ImuInterpolator::lerp_clamped(a, b, 3000);
    assert(after.timestamp_ns == b.timestamp_ns);  // clamped to b
    std::puts("  lerp_clamped boundary    OK");
}

// ── bspline: too few samples falls back to lerp ─────────────────────────────

static void test_bspline_fallback_2() {
    std::vector<ImuSample> block = {
        make_imu(1000, 0.0, 0.0, 9.81),
        make_imu(2000, 2.0, 0.0, 9.81),
    };
    auto result = ImuInterpolator::bspline(block, 1500);
    assert(result.timestamp_ns == 1500);
    assert(near(result.accel[0], 1.0));  // linear fallback
    std::puts("  bspline 2-sample fallback OK");
}

static void test_bspline_fallback_3() {
    std::vector<ImuSample> block = {
        make_imu(1000, 0.0, 0.0, 9.81),
        make_imu(2000, 1.0, 0.0, 9.81),
        make_imu(3000, 2.0, 0.0, 9.81),
    };
    auto result = ImuInterpolator::bspline(block, 1500);
    assert(result.timestamp_ns == 1500);
    assert(near(result.accel[0], 0.5, 1e-3));  // linear fallback
    std::puts("  bspline 3-sample fallback OK");
}

static void test_bspline_fallback_1() {
    std::vector<ImuSample> block = {
        make_imu(1000, 5.0, 0.0, 9.81),
    };
    auto result = ImuInterpolator::bspline(block, 1000);
    assert(result.timestamp_ns == 1000);
    assert(near(result.accel[0], 5.0));
    std::puts("  bspline 1-sample copy    OK");
}

// ── bspline: constant signal ────────────────────────────────────────────────

static void test_bspline_constant() {
    const int n = 10;
    std::vector<ImuSample> block;
    for (int i = 0; i < n; ++i) {
        block.push_back(make_imu(1000 + i * 100, 0.0, 0.0, 9.81));
    }

    // Evaluate at several points
    for (int i = 1; i < n - 1; ++i) {
        int64_t t = 1000 + i * 100 + 50;  // between samples
        auto result = ImuInterpolator::bspline(block, t);
        assert(result.timestamp_ns == t);
        assert(near(result.accel[2], 9.81, 0.05));
    }
    std::puts("  bspline constant signal  OK");
}

// ── bspline: linear signal reproduced ───────────────────────────────────────

static void test_bspline_linear() {
    const int n = 20;
    std::vector<ImuSample> block;
    for (int i = 0; i < n; ++i) {
        double t = static_cast<double>(i) / (n - 1);
        block.push_back(make_imu(
            static_cast<int64_t>(i * 1000000),  // 1ms apart
            3.0 * t,  // ax = 3t
            0.0, 9.81));
    }

    // B-spline should reproduce linear exactly within central region
    for (int i = 3; i < n - 3; ++i) {
        int64_t t = i * 1000000 + 500000;  // midpoint between samples
        auto result = ImuInterpolator::bspline(block, t);
        double expected = 3.0 * (static_cast<double>(t) /
                          static_cast<double>((n-1) * 1000000));
        assert(near(result.accel[0], expected, 0.1));
    }
    std::puts("  bspline linear signal    OK");
}

// ── bspline: sinusoidal signal smoother than lerp ───────────────────────────

static void test_bspline_vs_lerp_sinusoidal() {
    // Compare B-spline vs linear for a sinusoidal signal.
    // B-spline should have lower error overall.
    const int n = 50;  // 50 samples
    const double freq = 2.0;  // 2 Hz oscillation
    const double dt_ns = 1000000;  // 1ms spacing

    std::vector<ImuSample> block;
    for (int i = 0; i < n; ++i) {
        double t = static_cast<double>(i) * dt_ns / 1.0e9;
        double ax = std::sin(2.0 * std::numbers::pi * freq * t);
        block.push_back(make_imu(static_cast<int64_t>(i * dt_ns), ax, 0.0, 9.81));
    }

    double lerp_err_sum = 0;
    double bspline_err_sum = 0;
    int count = 0;

    // Evaluate at midpoints between samples
    for (int i = 5; i < n - 5; ++i) {
        int64_t t_ns = static_cast<int64_t>(i * dt_ns + dt_ns / 2);
        double t_sec = static_cast<double>(t_ns) / 1.0e9;
        double true_val = std::sin(2.0 * std::numbers::pi * freq * t_sec);

        // Linear
        auto lin = ImuInterpolator::lerp(block[i], block[i+1], t_ns);
        double lerp_err = std::abs(lin.accel[0] - true_val);

        // B-spline
        auto bsp = ImuInterpolator::bspline(block, t_ns);
        double bspline_err = std::abs(bsp.accel[0] - true_val);

        lerp_err_sum += lerp_err;
        bspline_err_sum += bspline_err;
        ++count;
    }

    double lerp_avg = lerp_err_sum / count;
    double bspline_avg = bspline_err_sum / count;

    // B-spline should be at least as good (usually significantly better)
    assert(bspline_avg <= lerp_avg + 0.01);

    std::printf("  bspline vs lerp sine     OK  (lerp=%.6f bspline=%.6f)\n",
                lerp_avg, bspline_avg);
}

// ── bspline: all channels interpolated ──────────────────────────────────────

static void test_bspline_all_channels() {
    const int n = 8;
    std::vector<ImuSample> block;
    for (int i = 0; i < n; ++i) {
        block.push_back(make_imu(
            1000 + i * 100,
            1.0, 2.0, 9.81,   // accel
            0.1, 0.2, 0.3));  // gyro
    }

    auto result = ImuInterpolator::bspline(block, 1350);
    assert(result.timestamp_ns == 1350);
    // All channels should be near their constant values
    assert(near(result.accel[0], 1.0, 0.1));
    assert(near(result.accel[1], 2.0, 0.1));
    assert(near(result.accel[2], 9.81, 0.1));
    assert(near(result.gyro[0], 0.1, 0.05));
    assert(near(result.gyro[1], 0.2, 0.05));
    assert(near(result.gyro[2], 0.3, 0.05));
    assert(near(result.temperature, 25.0, 0.5));
    std::puts("  bspline all channels     OK");
}

// ── Config flag exists ──────────────────────────────────────────────────────

static void test_bspline_config_flag() {
    SlamTimeSyncConfig cfg;
    assert(!cfg.use_bspline_interpolation);  // default off
    cfg.use_bspline_interpolation = true;
    assert(cfg.use_bspline_interpolation);
    std::puts("  Config flag              OK");
}

// ── Main ────────────────────────────────────────────────────────────────────

int main() {
    std::puts("ImuInterpolation:");
    test_lerp_midpoint();
    test_lerp_clamped_outside();
    test_bspline_fallback_1();
    test_bspline_fallback_2();
    test_bspline_fallback_3();
    test_bspline_constant();
    test_bspline_linear();
    test_bspline_vs_lerp_sinusoidal();
    test_bspline_all_channels();
    test_bspline_config_flag();
    std::puts("ImuInterpolation: ALL TESTS PASSED");
    return 0;
}
