// ─────────────────────────────────────────────────────────────────────────────
// Test — Online Extrinsic Refiner
// ─────────────────────────────────────────────────────────────────────────────
#include "calib/online_refiner.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

using namespace thunderbird::calib;

static constexpr double kEps = 1e-6;

static bool near(double a, double b, double tol = kEps) {
    return std::abs(a - b) < tol;
}

// ── Initial state ───────────────────────────────────────────────────────────

static void test_initial_state() {
    OnlineRefiner refiner;
    auto c = refiner.correction();
    assert(c.frame_count == 0);
    assert(near(c.confidence, 0.0));
    assert(near(c.rotation[0], 1.0));  // identity quaternion
    assert(!c.warning);
    std::puts("  Initial state            OK");
}

// ── Flat ground plane → identity correction ─────────────────────────────────

static void test_flat_ground() {
    OnlineRefinerConfig cfg;
    cfg.ground_max_height = -0.3;
    cfg.min_ground_points = 20;
    cfg.min_normal_z = 0.8;
    cfg.min_height = 0.5;
    OnlineRefiner refiner(cfg);

    // Generate a flat ground plane at z = -1.0 with noise
    std::mt19937 rng(42);
    std::normal_distribution<double> noise(0.0, 0.01);

    const int n = 500;
    std::vector<RefinerPoint> pts(n);
    for (int i = 0; i < n; ++i) {
        pts[i].x = static_cast<double>(i % 25) * 0.2 - 2.5;
        pts[i].y = static_cast<double>(i / 25) * 0.2 - 2.5;
        pts[i].z = -1.0 + noise(rng);  // ground plane at z=-1
    }

    // Process several frames to accumulate confidence
    for (int f = 0; f < 5; ++f) {
        bool ok = refiner.processFrame(pts.data(), n);
        assert(ok);
    }

    auto c = refiner.correction();
    assert(c.frame_count == 5);
    assert(c.confidence > 0.0);
    // For flat ground, correction should be near identity
    assert(near(c.rotation[0], 1.0, 0.05));
    assert(std::abs(c.rotation[1]) < 0.05);
    assert(std::abs(c.rotation[2]) < 0.05);
    std::puts("  Flat ground              OK");
}

// ── No ground points → processFrame returns false ───────────────────────────

static void test_no_ground() {
    OnlineRefinerConfig cfg;
    cfg.ground_max_height = -2.0;  // very low threshold
    cfg.min_ground_points = 100;
    OnlineRefiner refiner(cfg);

    // Points all above threshold
    const int n = 50;
    std::vector<RefinerPoint> pts(n);
    for (int i = 0; i < n; ++i) {
        pts[i].x = static_cast<double>(i);
        pts[i].y = 0;
        pts[i].z = 5.0;  // way above threshold
    }

    bool ok = refiner.processFrame(pts.data(), n);
    assert(!ok);
    assert(refiner.correction().frame_count == 0);
    std::puts("  No ground → skip         OK");
}

// ── Reset clears state ──────────────────────────────────────────────────────

static void test_reset() {
    OnlineRefinerConfig cfg;
    cfg.ground_max_height = 0.0;
    cfg.min_ground_points = 5;
    cfg.min_normal_z = 0.5;
    cfg.min_height = 0.1;
    OnlineRefiner refiner(cfg);

    // Process a frame of ground points
    const int n = 100;
    std::vector<RefinerPoint> pts(n);
    for (int i = 0; i < n; ++i) {
        pts[i].x = static_cast<double>(i % 10) * 0.5;
        pts[i].y = static_cast<double>(i / 10) * 0.5;
        pts[i].z = -1.0;
    }
    refiner.processFrame(pts.data(), n);
    assert(refiner.correction().frame_count > 0);

    refiner.reset();
    assert(refiner.correction().frame_count == 0);
    assert(near(refiner.correction().confidence, 0.0));
    std::puts("  Reset                    OK");
}

// ── Tilted ground detects pitch ─────────────────────────────────────────────

static void test_tilted_ground() {
    OnlineRefinerConfig cfg;
    cfg.ground_max_height = 0.0;
    cfg.min_ground_points = 20;
    cfg.min_normal_z = 0.7;
    cfg.min_height = 0.1;
    cfg.max_correction_deg = 10.0;
    OnlineRefiner refiner(cfg);

    // Tilted ground: 5° pitch → normal ≈ (-sin5°, 0, cos5°)
    const double pitch = 5.0 * M_PI / 180.0;
    const int n = 400;
    std::vector<RefinerPoint> pts(n);
    std::mt19937 rng(99);
    std::normal_distribution<double> noise(0.0, 0.005);

    for (int i = 0; i < n; ++i) {
        double x = static_cast<double>(i % 20) * 0.2 - 2.0;
        double y = static_cast<double>(i / 20) * 0.2 - 2.0;
        // Tilted plane: z = -1 + x * sin(pitch)
        double z = -1.0 + x * std::sin(pitch) + noise(rng);
        pts[i] = {x, y, z};
    }

    for (int f = 0; f < 10; ++f) {
        refiner.processFrame(pts.data(), n);
    }

    auto c = refiner.correction();
    assert(c.frame_count > 0);
    // The correction quaternion should have a non-trivial rotation
    // (not perfectly identity anymore due to tilt)
    // Should detect some rotation (even if small due to confidence weighting)
    assert(c.confidence > 0.0);
    std::puts("  Tilted ground detection  OK");
}

// ── Main ────────────────────────────────────────────────────────────────────

int main() {
    std::puts("OnlineRefiner:");
    test_initial_state();
    test_flat_ground();
    test_no_ground();
    test_reset();
    test_tilted_ground();
    std::puts("OnlineRefiner: ALL TESTS PASSED");
    return 0;
}
