// ─────────────────────────────────────────────────────────────────────────────
// Test — ClockService seed_offset() (Phase 8)
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/clock_service.h"

#include <cassert>
#include <cmath>
#include <cstdio>

using namespace thunderbird;

static constexpr double kEps = 1e-3;

static bool near(double a, double b, double tol = kEps) {
    return std::abs(a - b) < tol;
}

// ── Initial state: not calibrated ───────────────────────────────────────────

static void test_initial_uncalibrated() {
    ClockService cs;
    assert(!cs.is_calibrated());
    // hw_to_host returns input when not calibrated
    assert(cs.hw_to_host(1000) == 1000);
    std::puts("  Initial uncalibrated     OK");
}

// ── seed_offset: immediately calibrated ─────────────────────────────────────

static void test_seed_offset_calibrates() {
    ClockService cs;
    cs.seed_offset(5000.0);  // 5 µs offset

    assert(cs.is_calibrated());
    auto diag = cs.diagnostics();
    assert(near(diag.offset_ns, 5000.0));
    assert(diag.calibrated);
    std::puts("  seed_offset calibrates   OK");
}

// ── seed_offset: hw_to_host applies offset ──────────────────────────────────

static void test_seed_offset_applies() {
    ClockService cs;
    cs.seed_offset(1000000.0);  // 1ms offset

    // hw=0 → host=1ms
    int64_t host = cs.hw_to_host(0);
    assert(near(static_cast<double>(host), 1000000.0, 1.0));

    // hw=10ms → host=11ms
    host = cs.hw_to_host(10000000);
    assert(near(static_cast<double>(host), 11000000.0, 1.0));
    std::puts("  seed_offset applies      OK");
}

// ── seed_offset: negative offset (camera ahead of IMU) ─────────────────────

static void test_seed_negative_offset() {
    ClockService cs;
    cs.seed_offset(-2000000.0);  // -2ms

    int64_t host = cs.hw_to_host(10000000);
    assert(near(static_cast<double>(host), 8000000.0, 1.0));
    std::puts("  seed_offset negative     OK");
}

// ── seed_offset fires Calibrated event ──────────────────────────────────────

static void test_seed_fires_event() {
    ClockService cs;
    bool event_fired = false;
    cs.on_event([&](ClockEvent ev, const ClockDiagnostics&) {
        if (ev == ClockEvent::Calibrated)
            event_fired = true;
    });

    cs.seed_offset(1000.0);
    assert(event_fired);
    std::puts("  seed_offset event        OK");
}

// ── seed_offset then observe refines model ──────────────────────────────────

static void test_seed_then_observe() {
    ClockService cs;
    cs.seed_offset(100000.0);  // 100µs initial offset

    // Send observations that indicate slightly different offset
    // The model should adapt
    for (int i = 0; i < 20; ++i) {
        int64_t hw = static_cast<int64_t>(i) * 1000000;  // every 1ms
        int64_t host = hw + 100500;  // true offset is 100.5µs
        cs.observe(hw, host);
    }

    auto diag = cs.diagnostics();
    assert(diag.calibrated);
    // After observations, offset should converge toward 100500
    assert(near(diag.offset_ns, 100500.0, 1000.0));
    std::puts("  seed + observe refine    OK");
}

// ── reset clears seeded state ───────────────────────────────────────────────

static void test_reset_after_seed() {
    ClockService cs;
    cs.seed_offset(5000.0);
    assert(cs.is_calibrated());

    cs.reset();
    assert(!cs.is_calibrated());
    // hw_to_host should return input again
    assert(cs.hw_to_host(1000) == 1000);
    std::puts("  reset after seed         OK");
}

// ── host_to_hw inverse ──────────────────────────────────────────────────────

static void test_roundtrip_conversion() {
    ClockService cs;
    cs.seed_offset(50000.0);

    int64_t hw_in = 10000000;
    int64_t host = cs.hw_to_host(hw_in);
    int64_t hw_out = cs.host_to_hw(host);
    assert(std::abs(hw_out - hw_in) <= 1);  // roundtrip
    std::puts("  Roundtrip conversion     OK");
}

// ── Kalibr convention: offset_ns = -timeshift * 1e9 ─────────────────────────

static void test_kalibr_convention() {
    // Kalibr: t_imu = t_cam + timeshift
    // Thunderbird: camera_ts = imu_ts + offset_ns
    // Therefore: offset_ns = -timeshift * 1e9
    double kalibr_timeshift = 0.003;  // 3ms (camera behind IMU)
    double offset_ns = -kalibr_timeshift * 1.0e9;

    ClockService cs;
    cs.seed_offset(offset_ns);

    // Camera hw timestamp 10ms should map to host 7ms
    // (camera is 3ms behind, so host time = hw - 3ms)
    int64_t host = cs.hw_to_host(10000000);
    assert(near(static_cast<double>(host), 7000000.0, 2.0));
    std::puts("  Kalibr convention        OK");
}

// ── Main ────────────────────────────────────────────────────────────────────

int main() {
    std::puts("ClockServiceSeed:");
    test_initial_uncalibrated();
    test_seed_offset_calibrates();
    test_seed_offset_applies();
    test_seed_negative_offset();
    test_seed_fires_event();
    test_seed_then_observe();
    test_reset_after_seed();
    test_roundtrip_conversion();
    test_kalibr_convention();
    std::puts("ClockServiceSeed: ALL TESTS PASSED");
    return 0;
}
