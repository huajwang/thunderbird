// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Evaluation Framework: Trajectory Alignment Tests
// ─────────────────────────────────────────────────────────────────────────────
//
// Numerical sanity tests for Horn's SE(3)/Sim(3) alignment with outlier
// rejection.  Uses known toy trajectories so expected values can be verified
// analytically.
//
// Test plan:
//   1. Identity:          aligned_pair → should recover identity transform
//   2. Pure translation:  target = source + [3,4,5] → recover t=[3,4,5]
//   3. Pure rotation:     90° around Z → recover quaternion [cos45, 0, 0, sin45]
//   4. Rotation + trans:  combined SE(3) → verify roundtrip < 1e-10
//   5. Scale recovery:    Sim(3) with scale=2.5 → recover s≈2.5
//   6. Scale disabled:    same input but SE(3) → scale remains 1.0
//   7. Noise tolerance:   add 5 cm Gaussian noise → RMSE after < 0.1 m
//   8. Outlier rejection: 10% gross outliers → still recovers correct T
//   9. Circular trajectory: 50-point circle → SE(3) alignment exact
//  10. Degenerate (colinear): should still produce reasonable result
//
// ─────────────────────────────────────────────────────────────────────────────

#include "eval/trajectory_alignment.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace eval;
using Vec3  = std::array<double,3>;
using Quat4 = std::array<double,4>;

// ─── Helpers ─────────────────────────────────────────────────────────────────

static constexpr double TOL       = 1e-8;   // tight tolerance for exact cases
static constexpr double TOL_NOISE = 0.15;   // loose tolerance for noisy cases

static int g_pass = 0, g_fail = 0;

#define CHECK(cond, msg)                                                       \
    do {                                                                       \
        if (!(cond)) {                                                         \
            std::fprintf(stderr, "  FAIL: %s  (%s:%d)\n", msg, __FILE__,      \
                         __LINE__);                                            \
            ++g_fail;                                                          \
        } else {                                                               \
            ++g_pass;                                                          \
        }                                                                      \
    } while (0)

#define CHECK_NEAR(val, expected, tol, msg)                                    \
    CHECK(std::abs((val) - (expected)) < (tol), msg)

/// Build PointPairs from two matching vectors.
static std::vector<PointPair> makePairs(const std::vector<Vec3>& src,
                                         const std::vector<Vec3>& tgt) {
    assert(src.size() == tgt.size());
    std::vector<PointPair> pairs(src.size());
    for (size_t i = 0; i < src.size(); ++i) {
        pairs[i].source = src[i];
        pairs[i].target = tgt[i];
    }
    return pairs;
}

/// Rotate a vector by quaternion [w,x,y,z].
static Vec3 rotVec(const Quat4& q, const Vec3& v) {
    return horn_detail::rotateByQuat(q, v);
}

/// Generate points on a circle in the XY plane.
static std::vector<Vec3> circlePoints(int n, double radius) {
    std::vector<Vec3> pts;
    pts.reserve(n);
    for (int i = 0; i < n; ++i) {
        double theta = 2.0 * M_PI * static_cast<double>(i) / n;
        pts.push_back({radius * std::cos(theta),
                        radius * std::sin(theta),
                        0.0});
    }
    return pts;
}

/// Axis-angle to quaternion.
static Quat4 axisAngleToQuat(const Vec3& axis, double angle_rad) {
    double half = angle_rad / 2.0;
    double s = std::sin(half);
    double n = std::sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
    return {std::cos(half), s*axis[0]/n, s*axis[1]/n, s*axis[2]/n};
}

// ═════════════════════════════════════════════════════════════════════════════
//  Test 1: Identity alignment
// ═════════════════════════════════════════════════════════════════════════════

static void test_identity() {
    std::fprintf(stderr, "[test] Identity alignment...\n");

    auto pts = circlePoints(20, 5.0);
    auto pairs = makePairs(pts, pts);  // source == target

    auto r = alignTrajectories(pairs);

    CHECK_NEAR(r.rotation[0], 1.0, TOL, "qw ≈ 1");
    CHECK_NEAR(r.rotation[1], 0.0, TOL, "qx ≈ 0");
    CHECK_NEAR(r.rotation[2], 0.0, TOL, "qy ≈ 0");
    CHECK_NEAR(r.rotation[3], 0.0, TOL, "qz ≈ 0");
    CHECK_NEAR(r.translation[0], 0.0, TOL, "tx ≈ 0");
    CHECK_NEAR(r.translation[1], 0.0, TOL, "ty ≈ 0");
    CHECK_NEAR(r.translation[2], 0.0, TOL, "tz ≈ 0");
    CHECK_NEAR(r.scale, 1.0, TOL, "scale ≈ 1");
    CHECK_NEAR(r.rmse_after, 0.0, TOL, "RMSE ≈ 0");
}

// ═════════════════════════════════════════════════════════════════════════════
//  Test 2: Pure translation
// ═════════════════════════════════════════════════════════════════════════════

static void test_pure_translation() {
    std::fprintf(stderr, "[test] Pure translation [3,4,5]...\n");

    Vec3 t = {3.0, 4.0, 5.0};
    auto src = circlePoints(20, 5.0);
    std::vector<Vec3> tgt;
    tgt.reserve(src.size());
    for (const auto& p : src) {
        tgt.push_back({p[0]+t[0], p[1]+t[1], p[2]+t[2]});
    }

    auto pairs = makePairs(src, tgt);
    auto r = alignTrajectories(pairs);

    CHECK_NEAR(r.rotation[0], 1.0, TOL, "qw ≈ 1 (no rotation)");
    CHECK_NEAR(r.translation[0], 3.0, TOL, "tx ≈ 3");
    CHECK_NEAR(r.translation[1], 4.0, TOL, "ty ≈ 4");
    CHECK_NEAR(r.translation[2], 5.0, TOL, "tz ≈ 5");
    CHECK_NEAR(r.rmse_after, 0.0, TOL, "RMSE ≈ 0");
}

// ═════════════════════════════════════════════════════════════════════════════
//  Test 3: Pure rotation (90° around Z)
// ═════════════════════════════════════════════════════════════════════════════

static void test_pure_rotation_z90() {
    std::fprintf(stderr, "[test] Pure rotation 90° around Z...\n");

    // 90° around Z: q = [cos(45°), 0, 0, sin(45°)]
    Quat4 q_expected = axisAngleToQuat({0,0,1}, M_PI/2.0);

    // Use a 3D trajectory (not just XY circle, add Z variation).
    std::vector<Vec3> src;
    for (int i = 0; i < 30; ++i) {
        double t = static_cast<double>(i) * 0.1;
        src.push_back({std::cos(t) * 5.0,
                        std::sin(t) * 3.0,
                        std::sin(t * 0.7) * 2.0});
    }

    std::vector<Vec3> tgt;
    tgt.reserve(src.size());
    for (const auto& p : src) {
        tgt.push_back(rotVec(q_expected, p));
    }

    auto pairs = makePairs(src, tgt);
    auto r = alignTrajectories(pairs);

    // Quaternion sign ambiguity: both q and -q represent same rotation.
    double q_dot = r.rotation[0]*q_expected[0] + r.rotation[1]*q_expected[1] +
                   r.rotation[2]*q_expected[2] + r.rotation[3]*q_expected[3];
    CHECK_NEAR(std::abs(q_dot), 1.0, TOL, "quaternion matches expected");
    CHECK_NEAR(r.rmse_after, 0.0, TOL, "RMSE ≈ 0");
}

// ═════════════════════════════════════════════════════════════════════════════
//  Test 4: Combined rotation + translation
// ═════════════════════════════════════════════════════════════════════════════

static void test_rotation_plus_translation() {
    std::fprintf(stderr, "[test] SE(3): rotation + translation...\n");

    // 45° around axis [1,1,0] + translation [10, -3, 7].
    Quat4 q_expected = axisAngleToQuat({1,1,0}, M_PI/4.0);
    Vec3  t_expected = {10.0, -3.0, 7.0};

    std::vector<Vec3> src;
    for (int i = 0; i < 50; ++i) {
        double t = static_cast<double>(i) * 0.5;
        src.push_back({t, std::sin(t)*2.0, std::cos(t)*1.5});
    }

    std::vector<Vec3> tgt;
    tgt.reserve(src.size());
    for (const auto& p : src) {
        auto rot = rotVec(q_expected, p);
        tgt.push_back({rot[0]+t_expected[0],
                        rot[1]+t_expected[1],
                        rot[2]+t_expected[2]});
    }

    auto pairs = makePairs(src, tgt);
    auto r = alignTrajectories(pairs);

    // Verify roundtrip: aligned source should match target.
    auto aligned = applyAlignment(src, r);
    double max_err = 0.0;
    for (size_t i = 0; i < src.size(); ++i) {
        double dx = aligned[i][0]-tgt[i][0];
        double dy = aligned[i][1]-tgt[i][1];
        double dz = aligned[i][2]-tgt[i][2];
        double err = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (err > max_err) max_err = err;
    }

    CHECK(max_err < 1e-9, "roundtrip error < 1e-9");
    CHECK_NEAR(r.rmse_after, 0.0, TOL, "RMSE ≈ 0");
    CHECK_NEAR(r.scale, 1.0, TOL, "scale ≈ 1 (SE(3))");
}

// ═════════════════════════════════════════════════════════════════════════════
//  Test 5: Scale recovery (Sim(3))
// ═════════════════════════════════════════════════════════════════════════════

static void test_scale_recovery() {
    std::fprintf(stderr, "[test] Sim(3): scale recovery (s=2.5)...\n");

    double s_expected = 2.5;
    Quat4  q_expected = axisAngleToQuat({0,1,0}, M_PI/6.0);  // 30° around Y
    Vec3   t_expected = {1.0, 2.0, 3.0};

    auto src = circlePoints(40, 5.0);
    // Add Z variation for 3D spread.
    for (size_t i = 0; i < src.size(); ++i) {
        src[i][2] = std::sin(static_cast<double>(i) * 0.3) * 3.0;
    }

    std::vector<Vec3> tgt;
    tgt.reserve(src.size());
    for (const auto& p : src) {
        auto rot = rotVec(q_expected, p);
        tgt.push_back({s_expected*rot[0] + t_expected[0],
                        s_expected*rot[1] + t_expected[1],
                        s_expected*rot[2] + t_expected[2]});
    }

    auto pairs = makePairs(src, tgt);

    // With scale estimation.
    AlignmentConfig cfg;
    cfg.estimate_scale = true;
    auto r = alignTrajectories(pairs, cfg);

    CHECK_NEAR(r.scale, s_expected, 1e-6, "scale ≈ 2.5");
    CHECK_NEAR(r.rmse_after, 0.0, 1e-6, "RMSE ≈ 0");

    std::fprintf(stderr, "    recovered scale = %.10f (expected %.1f)\n",
                 r.scale, s_expected);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Test 6: Scale disabled (SE(3) on scaled data)
// ═════════════════════════════════════════════════════════════════════════════

static void test_scale_disabled() {
    std::fprintf(stderr, "[test] SE(3) on scaled data (scale fixed at 1.0)...\n");

    double s_applied = 2.0;
    auto src = circlePoints(30, 3.0);
    for (size_t i = 0; i < src.size(); ++i) {
        src[i][2] = static_cast<double>(i) * 0.1;
    }

    std::vector<Vec3> tgt;
    for (const auto& p : src) {
        tgt.push_back({s_applied * p[0], s_applied * p[1], s_applied * p[2]});
    }

    auto pairs = makePairs(src, tgt);

    // Without scale estimation.
    AlignmentConfig cfg;
    cfg.estimate_scale = false;
    auto r = alignTrajectories(pairs, cfg);

    CHECK_NEAR(r.scale, 1.0, TOL, "scale = 1.0 (forced)");
    CHECK(r.rmse_after > 0.5, "RMSE > 0 (can't fix scale without Sim(3))");
    std::fprintf(stderr, "    RMSE with forced scale=1: %.4f m\n", r.rmse_after);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Test 7: Noise tolerance
// ═════════════════════════════════════════════════════════════════════════════

static void test_noise_tolerance() {
    std::fprintf(stderr, "[test] Noise tolerance (5 cm Gaussian)...\n");

    Quat4 q_true = axisAngleToQuat({1,0,0}, M_PI/3.0);  // 60° around X
    Vec3  t_true = {5.0, -2.0, 1.0};

    std::vector<Vec3> src;
    for (int i = 0; i < 200; ++i) {
        double t = static_cast<double>(i) * 0.3;
        src.push_back({t, std::sin(t) * 5.0, std::cos(t * 0.5) * 3.0});
    }

    std::mt19937 rng(42);
    std::normal_distribution<double> noise(0.0, 0.05);  // 5 cm σ

    std::vector<Vec3> tgt;
    for (const auto& p : src) {
        auto rot = rotVec(q_true, p);
        tgt.push_back({rot[0] + t_true[0] + noise(rng),
                        rot[1] + t_true[1] + noise(rng),
                        rot[2] + t_true[2] + noise(rng)});
    }

    auto pairs = makePairs(src, tgt);
    auto r = alignTrajectories(pairs);

    CHECK(r.rmse_after < TOL_NOISE, "RMSE after alignment < 0.15 m");
    CHECK(r.rmse_after < r.rmse_before, "alignment improved RMSE");

    // Recovered translation should be close.
    double t_err = std::sqrt(
        (r.translation[0]-t_true[0])*(r.translation[0]-t_true[0]) +
        (r.translation[1]-t_true[1])*(r.translation[1]-t_true[1]) +
        (r.translation[2]-t_true[2])*(r.translation[2]-t_true[2]));
    CHECK(t_err < 0.15, "translation error < 15 cm");

    std::fprintf(stderr, "    RMSE: %.6f m,  translation error: %.6f m\n",
                 r.rmse_after, t_err);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Test 8: Outlier rejection
// ═════════════════════════════════════════════════════════════════════════════

static void test_outlier_rejection() {
    std::fprintf(stderr, "[test] Outlier rejection (10%% gross outliers)...\n");

    Quat4 q_true = axisAngleToQuat({0,0,1}, M_PI/4.0);  // 45° around Z
    Vec3  t_true = {2.0, 3.0, 0.5};

    std::vector<Vec3> src;
    for (int i = 0; i < 100; ++i) {
        double t = static_cast<double>(i) * 0.2;
        src.push_back({t * 0.5, std::sin(t) * 4.0, std::cos(t) * 2.0});
    }

    std::mt19937 rng(123);
    std::normal_distribution<double> noise(0.0, 0.01);    // 1 cm inlier noise
    std::uniform_real_distribution<double> gross(-50, 50); // gross outlier

    std::vector<Vec3> tgt;
    int outlier_count = 0;
    for (size_t i = 0; i < src.size(); ++i) {
        auto rot = rotVec(q_true, src[i]);
        Vec3 tgt_pt = {rot[0]+t_true[0]+noise(rng),
                        rot[1]+t_true[1]+noise(rng),
                        rot[2]+t_true[2]+noise(rng)};

        // 10% outliers: replace with random garbage.
        if (i % 10 == 5) {
            tgt_pt = {gross(rng), gross(rng), gross(rng)};
            ++outlier_count;
        }
        tgt.push_back(tgt_pt);
    }

    auto pairs = makePairs(src, tgt);

    // Without outlier rejection — should be significantly worse.
    AlignmentConfig cfg_no_reject;
    auto r_no = alignTrajectories(pairs, cfg_no_reject);

    // With outlier rejection.
    AlignmentConfig cfg_reject;
    cfg_reject.outlier_rejection  = true;
    cfg_reject.outlier_threshold_sigma = 3.0;
    cfg_reject.max_outlier_iters  = 15;
    auto r_rej = alignTrajectories(pairs, cfg_reject);

    CHECK(r_rej.rmse_after < r_no.rmse_after,
          "outlier rejection improves RMSE");
    CHECK(r_rej.outlier_count >= static_cast<size_t>(outlier_count * 0.7),
          "detected ≥70% of outliers");
    CHECK(r_rej.converged, "outlier rejection converged");

    // Translation should be accurate with rejection.
    double t_err = std::sqrt(
        (r_rej.translation[0]-t_true[0])*(r_rej.translation[0]-t_true[0]) +
        (r_rej.translation[1]-t_true[1])*(r_rej.translation[1]-t_true[1]) +
        (r_rej.translation[2]-t_true[2])*(r_rej.translation[2]-t_true[2]));
    CHECK(t_err < 0.2, "translation error < 20 cm with outlier rejection");

    std::fprintf(stderr, "    without rejection: RMSE=%.4f, with: RMSE=%.4f\n",
                 r_no.rmse_after, r_rej.rmse_after);
    std::fprintf(stderr, "    outliers detected: %zu/%d,  translation err: %.4f m\n",
                 r_rej.outlier_count, outlier_count, t_err);
    std::fprintf(stderr, "    converged in %zu iterations\n", r_rej.iterations);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Test 9: Circular trajectory (50 points)
// ═════════════════════════════════════════════════════════════════════════════

static void test_circular_trajectory() {
    std::fprintf(stderr, "[test] Circular trajectory (50 pts, 3D)...\n");

    auto src = circlePoints(50, 10.0);
    // Elevate with Z to make it 3D (helix).
    for (size_t i = 0; i < src.size(); ++i) {
        src[i][2] = static_cast<double>(i) * 0.15;
    }

    // Apply known SE(3): 30° around [1, -1, 2] + t=[7,8,9].
    Quat4 q = axisAngleToQuat({1,-1,2}, M_PI/6.0);
    Vec3  t = {7.0, 8.0, 9.0};

    std::vector<Vec3> tgt;
    tgt.reserve(src.size());
    for (const auto& p : src) {
        auto rot = rotVec(q, p);
        tgt.push_back({rot[0]+t[0], rot[1]+t[1], rot[2]+t[2]});
    }

    auto pairs = makePairs(src, tgt);
    auto r = alignTrajectories(pairs);

    // Roundtrip.
    auto aligned = applyAlignment(src, r);
    double max_err = 0.0;
    for (size_t i = 0; i < src.size(); ++i) {
        double dx = aligned[i][0]-tgt[i][0];
        double dy = aligned[i][1]-tgt[i][1];
        double dz = aligned[i][2]-tgt[i][2];
        double err = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (err > max_err) max_err = err;
    }

    CHECK(max_err < 1e-9, "circular traj roundtrip < 1e-9");
    CHECK_NEAR(r.rmse_after, 0.0, TOL, "RMSE ≈ 0");
}

// ═════════════════════════════════════════════════════════════════════════════
//  Test 10: Degenerate (near-colinear) points
// ═════════════════════════════════════════════════════════════════════════════

static void test_degenerate() {
    std::fprintf(stderr, "[test] Degenerate near-colinear trajectory...\n");

    // Points nearly on a line in X with tiny Y/Z perturbation.
    std::vector<Vec3> src;
    for (int i = 0; i < 20; ++i) {
        double x = static_cast<double>(i) * 1.0;
        src.push_back({x, 0.001 * std::sin(x), 0.001 * std::cos(x)});
    }

    // Apply translation only (rotation hard to determine from line).
    Vec3 t = {5.0, 3.0, 1.0};
    std::vector<Vec3> tgt;
    for (const auto& p : src) {
        tgt.push_back({p[0]+t[0], p[1]+t[1], p[2]+t[2]});
    }

    auto pairs = makePairs(src, tgt);
    auto r = alignTrajectories(pairs);

    // Should at least recover translation well.
    CHECK_NEAR(r.translation[0], t[0], 0.1, "tx ≈ 5 (degenerate)");
    CHECK_NEAR(r.translation[1], t[1], 0.1, "ty ≈ 3 (degenerate)");
    CHECK_NEAR(r.translation[2], t[2], 0.1, "tz ≈ 1 (degenerate)");
    CHECK(r.rmse_after < 0.01, "RMSE reasonable for degenerate case");

    std::fprintf(stderr, "    RMSE: %.8f m\n", r.rmse_after);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Test 11: Numerical sanity — known exact values
// ═════════════════════════════════════════════════════════════════════════════
//
// Verifiable by hand computation:
//
//   Source points:  (1,0,0), (0,1,0), (0,0,1), (1,1,0), (1,0,1)
//   Transform:     90° rotation around Z, then translate by (10, 20, 30)
//
//   R_z(90°) · [1,0,0] = [0,1,0]  →  target = [10, 21, 30]
//   R_z(90°) · [0,1,0] = [-1,0,0] →  target = [9,  20, 30]
//   R_z(90°) · [0,0,1] = [0,0,1]  →  target = [10, 20, 31]
//   R_z(90°) · [1,1,0] = [-1,1,0] →  target = [9,  21, 30]
//   R_z(90°) · [1,0,1] = [0,1,1]  →  target = [10, 21, 31]

static void test_numerical_sanity() {
    std::fprintf(stderr, "[test] Numerical sanity (hand-verified transform)...\n");

    std::vector<Vec3> src = {
        {1,0,0}, {0,1,0}, {0,0,1}, {1,1,0}, {1,0,1}
    };
    std::vector<Vec3> tgt = {
        {10, 21, 30}, {9, 20, 30}, {10, 20, 31}, {9, 21, 30}, {10, 21, 31}
    };

    auto pairs = makePairs(src, tgt);
    auto r = alignTrajectories(pairs);

    // Expected quaternion for 90° around Z: [cos45°, 0, 0, sin45°]
    // = [0.70711, 0, 0, 0.70711]
    double cos45 = std::cos(M_PI/4.0);

    std::fprintf(stderr, "    rotation:    [%.8f, %.8f, %.8f, %.8f]\n",
                 r.rotation[0], r.rotation[1], r.rotation[2], r.rotation[3]);
    std::fprintf(stderr, "    expected:    [%.8f, 0, 0, %.8f]\n", cos45, cos45);
    std::fprintf(stderr, "    translation: [%.8f, %.8f, %.8f]\n",
                 r.translation[0], r.translation[1], r.translation[2]);
    std::fprintf(stderr, "    expected:    [10, 20, 30]\n");
    std::fprintf(stderr, "    scale:       %.10f\n", r.scale);
    std::fprintf(stderr, "    RMSE after:  %.2e\n", r.rmse_after);

    CHECK_NEAR(r.rotation[0], cos45, TOL, "qw ≈ cos(45°)");
    CHECK_NEAR(std::abs(r.rotation[1]), 0.0, TOL, "qx ≈ 0");
    CHECK_NEAR(std::abs(r.rotation[2]), 0.0, TOL, "qy ≈ 0");
    CHECK_NEAR(r.rotation[3], cos45, TOL, "qz ≈ sin(45°)");
    CHECK_NEAR(r.translation[0], 10.0, TOL, "tx = 10");
    CHECK_NEAR(r.translation[1], 20.0, TOL, "ty = 20");
    CHECK_NEAR(r.translation[2], 30.0, TOL, "tz = 30");
    CHECK_NEAR(r.rmse_after, 0.0, TOL, "RMSE = 0");
}

// ═════════════════════════════════════════════════════════════════════════════
//  Main
// ═════════════════════════════════════════════════════════════════════════════

int main() {
    std::fprintf(stderr, "\n══════════════════════════════════════════════\n");
    std::fprintf(stderr, "  Trajectory Alignment — Numerical Sanity Tests\n");
    std::fprintf(stderr, "══════════════════════════════════════════════\n\n");

    test_identity();
    test_pure_translation();
    test_pure_rotation_z90();
    test_rotation_plus_translation();
    test_scale_recovery();
    test_scale_disabled();
    test_noise_tolerance();
    test_outlier_rejection();
    test_circular_trajectory();
    test_degenerate();
    test_numerical_sanity();

    std::fprintf(stderr, "\n──────────────────────────────────────────────\n");
    std::fprintf(stderr, "  Results: %d passed, %d failed\n", g_pass, g_fail);
    std::fprintf(stderr, "──────────────────────────────────────────────\n\n");

    return g_fail > 0 ? 1 : 0;
}
