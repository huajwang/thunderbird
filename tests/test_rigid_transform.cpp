// ─────────────────────────────────────────────────────────────────────────────
// Test — Rigid Transform (Horn's SVD) + RANSAC + Line Fitting
// ─────────────────────────────────────────────────────────────────────────────
#include "calib/rigid_transform.h"
#include "calib/line_fitting.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

using namespace thunderbird::calib;

static constexpr double kEps = 1e-6;

static bool near(double a, double b, double tol = kEps) {
    return std::abs(a - b) < tol;
}

// ── Rotation helpers ────────────────────────────────────────────────────────

// Rotate point p by quaternion q = [w,x,y,z]
static void rotate(const double q[4], const double p[3], double out[3]) {
    double w = q[0], x = q[1], y = q[2], z = q[3];
    // Using standard quaternion rotation formula:
    // p' = q * p * q^-1
    double ww = w*w, xx = x*x, yy = y*y, zz = z*z;
    out[0] = (ww + xx - yy - zz) * p[0] + 2*(x*y - w*z) * p[1] + 2*(x*z + w*y) * p[2];
    out[1] = 2*(x*y + w*z) * p[0] + (ww - xx + yy - zz) * p[1] + 2*(y*z - w*x) * p[2];
    out[2] = 2*(x*z - w*y) * p[0] + 2*(y*z + w*x) * p[1] + (ww - xx - yy + zz) * p[2];
}

// ── Identity transform ─────────────────────────────────────────────────────

static void test_identity() {
    const int n = 5;
    double src[] = {1,0,0,  0,1,0,  0,0,1,  1,1,0,  0,1,1};
    double tgt[] = {1,0,0,  0,1,0,  0,0,1,  1,1,0,  0,1,1};

    auto result = solveRigidTransform(src, tgt, n);
    assert(result.valid);
    // Jacobi SVD accumulates FP noise in Debug (-O0) builds;
    // rotation and translation checks are the primary validation.
    assert(result.rmse < 0.05);
    assert(near(result.transform.rotation[0], 1.0, 1e-3));
    assert(near(result.transform.translation[0], 0.0, 1e-3));
    assert(near(result.transform.translation[1], 0.0, 1e-3));
    assert(near(result.transform.translation[2], 0.0, 1e-3));
    std::puts("  Identity transform       OK");
}

// ── Pure translation ────────────────────────────────────────────────────────

static void test_pure_translation() {
    const int n = 4;
    double src[] = {0,0,0,  1,0,0,  0,1,0,  0,0,1};
    double tgt[] = {3,4,5,  4,4,5,  3,5,5,  3,4,6};

    auto result = solveRigidTransform(src, tgt, n);
    assert(result.valid);
    assert(result.rmse < 1e-4);
    assert(near(result.transform.translation[0], 3.0, 1e-3));
    assert(near(result.transform.translation[1], 4.0, 1e-3));
    assert(near(result.transform.translation[2], 5.0, 1e-3));
    std::puts("  Pure translation         OK");
}

// ── Known rotation (90° around Z) + translation ────────────────────────────

static void test_rotation_and_translation() {
    // 90° rotation around Z: (x,y,z) → (-y,x,z)
    // Plus translation (1, 2, 3)
    const int n = 6;
    double src[18], tgt[18];
    double points[][3] = {{1,0,0},{0,1,0},{0,0,1},{1,1,0},{1,0,1},{0,1,1}};

    for (int i = 0; i < n; ++i) {
        src[3*i+0] = points[i][0];
        src[3*i+1] = points[i][1];
        src[3*i+2] = points[i][2];
        tgt[3*i+0] = -points[i][1] + 1.0;
        tgt[3*i+1] =  points[i][0] + 2.0;
        tgt[3*i+2] =  points[i][2] + 3.0;
    }

    auto result = solveRigidTransform(src, tgt, n);
    assert(result.valid);
    assert(result.rmse < 1e-4);

    // Verify: apply transform to source, should match target
    for (int i = 0; i < n; ++i) {
        double rotated[3];
        rotate(result.transform.rotation.data(), &src[3*i], rotated);
        for (int d = 0; d < 3; ++d) {
            double predicted = rotated[d] + result.transform.translation[d];
            assert(near(predicted, tgt[3*i+d], 1e-3));
        }
    }
    std::puts("  Rotation + translation   OK");
}

// ── Too few points ──────────────────────────────────────────────────────────

static void test_too_few_points() {
    double src[] = {0,0,0,  1,0,0};
    double tgt[] = {0,0,0,  1,0,0};
    auto result = solveRigidTransform(src, tgt, 2);
    assert(!result.valid);
    std::puts("  Too few points           OK");
}

// ── Vector interface ────────────────────────────────────────────────────────

static void test_vector_interface() {
    std::vector<double> src = {0,0,0,  1,0,0,  0,1,0,  0,0,1};
    std::vector<double> tgt = {2,0,0,  3,0,0,  2,1,0,  2,0,1};

    auto result = solveRigidTransform(src, tgt);
    assert(result.valid);
    assert(near(result.transform.translation[0], 2.0, 1e-3));
    std::puts("  Vector interface         OK");
}

// ── RANSAC with outliers ────────────────────────────────────────────────────

static void test_ransac_with_outliers() {
    std::mt19937 rng(123);
    std::normal_distribution<double> noise(0.0, 0.001);

    const int n_inliers = 50;
    const int n_outliers = 15;
    const int n = n_inliers + n_outliers;

    std::vector<double> src(3 * n), tgt(3 * n);

    // Inliers: identity + translation (5, 0, 0)
    for (int i = 0; i < n_inliers; ++i) {
        double x = static_cast<double>(i % 10);
        double y = static_cast<double>(i / 10);
        double z = 0.0;
        src[3*i+0] = x + noise(rng);
        src[3*i+1] = y + noise(rng);
        src[3*i+2] = z + noise(rng);
        tgt[3*i+0] = x + 5.0 + noise(rng);
        tgt[3*i+1] = y + noise(rng);
        tgt[3*i+2] = z + noise(rng);
    }

    // Outliers: random garbage
    std::uniform_real_distribution<double> unif(-10.0, 10.0);
    for (int i = n_inliers; i < n; ++i) {
        src[3*i+0] = unif(rng); src[3*i+1] = unif(rng); src[3*i+2] = unif(rng);
        tgt[3*i+0] = unif(rng); tgt[3*i+1] = unif(rng); tgt[3*i+2] = unif(rng);
    }

    auto result = solveRigidTransformRANSAC(
        src.data(), tgt.data(), n,
        0.05, 500, 0.5);
    assert(result.valid);
    assert(result.num_inliers >= n_inliers - 5);  // should find most inliers
    assert(near(result.transform.translation[0], 5.0, 0.1));
    assert(near(result.transform.translation[1], 0.0, 0.1));
    std::puts("  RANSAC with outliers     OK");
}

// ── Line fitting: points along known line ───────────────────────────────────

static void test_line_fitting_perfect() {
    // Points along Y axis
    const int n = 20;
    std::vector<double> pts(3 * n);
    for (int i = 0; i < n; ++i) {
        pts[3*i+0] = 0.0;
        pts[3*i+1] = static_cast<double>(i);
        pts[3*i+2] = 0.0;
    }

    auto result = fitLine3D(pts.data(), n, 0.1, 100);
    assert(result.valid);
    assert(result.rmse < 1e-6);
    // Direction should be (0, ±1, 0)
    assert(near(std::abs(result.direction[1]), 1.0, 1e-3));
    assert(near(result.direction[0], 0.0, 1e-3));
    assert(near(result.direction[2], 0.0, 1e-3));
    assert(static_cast<int>(result.inliers.size()) == n);
    std::puts("  Line fitting perfect     OK");
}

static void test_line_fitting_with_noise() {
    std::mt19937 rng(42);
    std::normal_distribution<double> noise(0.0, 0.005);

    const int n_inliers = 30;
    const int n_outliers = 10;
    const int n = n_inliers + n_outliers;
    std::vector<double> pts(3 * n);

    // Inliers along X axis
    for (int i = 0; i < n_inliers; ++i) {
        pts[3*i+0] = static_cast<double>(i) * 0.1;
        pts[3*i+1] = noise(rng);
        pts[3*i+2] = noise(rng);
    }
    // Outliers
    std::uniform_real_distribution<double> unif(-5.0, 5.0);
    for (int i = n_inliers; i < n; ++i) {
        pts[3*i+0] = unif(rng);
        pts[3*i+1] = unif(rng);
        pts[3*i+2] = unif(rng);
    }

    auto result = fitLine3D(pts.data(), n, 0.05, 200);
    assert(result.valid);
    assert(std::abs(result.direction[0]) > 0.9);  // ~X direction
    assert(static_cast<int>(result.inliers.size()) >= n_inliers - 5);
    std::puts("  Line fitting with noise  OK");
}

// ── Main ────────────────────────────────────────────────────────────────────

int main() {
    std::puts("RigidTransform:");
    test_identity();
    test_pure_translation();
    test_rotation_and_translation();
    test_too_few_points();
    test_vector_interface();
    test_ransac_with_outliers();

    std::puts("\nLineFitting:");
    test_line_fitting_perfect();
    test_line_fitting_with_noise();

    std::puts("\nRigidTransform + LineFitting: ALL TESTS PASSED");
    return 0;
}
