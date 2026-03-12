// ─────────────────────────────────────────────────────────────────────────────
// Test — Uniform Cubic B-Spline
// ─────────────────────────────────────────────────────────────────────────────
#include "calib/bspline.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace thunderbird::calib;

static constexpr double kEps = 1e-6;

static bool near(double a, double b, double tol = kEps) {
    return std::abs(a - b) < tol;
}

// ── Construction ────────────────────────────────────────────────────────────

static void test_construction() {
    UniformCubicBSpline sp(3);
    assert(sp.dim() == 3);
    assert(!sp.isValid());
    std::puts("  Construction             OK");
}

// ── initFromData with constant signal ───────────────────────────────────────

static void test_constant_signal() {
    // A constant signal should be reproduced exactly.
    const int n = 10;
    std::vector<double> data(n, 42.0);

    UniformCubicBSpline sp(1);
    bool ok = sp.initFromData(0.0, 1.0, data.data(), n);
    assert(ok);
    assert(sp.isValid());
    assert(sp.numControlPoints() == n);

    double val = 0;
    // Evaluate at several points in valid interval
    for (double t = sp.tMin(); t <= sp.tMax(); t += 0.07) {
        ok = sp.eval(t, &val);
        assert(ok);
        assert(near(val, 42.0, 1e-3));
    }
    std::puts("  Constant signal          OK");
}

// ── initFromData with linear signal ─────────────────────────────────────────

static void test_linear_signal() {
    // A cubic B-spline should reproduce a linear signal exactly
    // (within the valid interval).
    const int n = 20;
    std::vector<double> data(n);
    for (int i = 0; i < n; ++i)
        data[i] = 3.0 * (static_cast<double>(i) / (n - 1));

    UniformCubicBSpline sp(1);
    sp.initFromData(0.0, 1.0, data.data(), n);
    assert(sp.isValid());

    // Evaluate at midpoints
    double dt = 1.0 / (n - 1);
    for (int i = 2; i < n - 2; ++i) {
        double t = dt * i + dt * 0.5;
        if (t < sp.tMin() || t > sp.tMax()) continue;
        double val;
        sp.eval(t, &val);
        double expected = 3.0 * t;
        assert(near(val, expected, 0.05));
    }
    std::puts("  Linear signal            OK");
}

// ── Derivative of linear signal ─────────────────────────────────────────────

static void test_linear_derivative() {
    // Derivative of linear function f(t)=3t should be ~3.
    const int n = 20;
    double dt_knot = 1.0 / (n - 1);
    std::vector<double> data(n);
    for (int i = 0; i < n; ++i)
        data[i] = 3.0 * (dt_knot * i);

    UniformCubicBSpline sp(1);
    sp.initFromData(0.0, 1.0, data.data(), n);

    double deriv;
    double t = (sp.tMin() + sp.tMax()) / 2.0;
    bool ok = sp.evalDerivative(t, 1, &deriv);
    assert(ok);
    assert(near(deriv, 3.0, 0.1));
    std::puts("  Linear derivative        OK");
}

// ── Multidimensional (3D) ───────────────────────────────────────────────────

static void test_3d_spline() {
    const int n = 10;
    std::vector<double> data(n * 3);
    for (int i = 0; i < n; ++i) {
        double t = static_cast<double>(i) / (n - 1);
        data[3 * i + 0] = t;           // x = t
        data[3 * i + 1] = t * t;       // y = t²
        data[3 * i + 2] = 1.0;         // z = 1
    }

    UniformCubicBSpline sp(3);
    sp.initFromData(0.0, 1.0, data.data(), n);
    assert(sp.isValid());
    assert(sp.dim() == 3);

    double result[3];
    double midT = (sp.tMin() + sp.tMax()) / 2.0;
    bool ok = sp.eval(midT, result);
    assert(ok);
    // z should be very close to 1.0
    assert(near(result[2], 1.0, 0.05));
    std::puts("  3D spline                OK");
}

// ── Out-of-range eval returns false ─────────────────────────────────────────

static void test_out_of_range() {
    const int n = 5;
    std::vector<double> data(n, 1.0);

    UniformCubicBSpline sp(1);
    sp.initFromData(0.0, 1.0, data.data(), n);

    double val;
    // Well before valid range
    bool ok = sp.eval(-10.0, &val);
    assert(!ok);
    // Well after valid range
    ok = sp.eval(10.0, &val);
    assert(!ok);
    std::puts("  Out-of-range boundary    OK");
}

// ── Second derivative ───────────────────────────────────────────────────────

static void test_second_derivative() {
    // For a quadratic signal f(t) = t², second derivative = 2.
    const int n = 30;
    std::vector<double> data(n);
    double dt_knot = 1.0 / (n - 1);
    for (int i = 0; i < n; ++i) {
        double t = dt_knot * i;
        data[i] = t * t;
    }

    UniformCubicBSpline sp(1);
    sp.initFromData(0.0, 1.0, data.data(), n);

    double d2;
    double midT = (sp.tMin() + sp.tMax()) / 2.0;
    bool ok = sp.evalDerivative(midT, 2, &d2);
    assert(ok);
    assert(near(d2, 2.0, 0.3));
    std::puts("  Second derivative        OK");
}

// ── initFromCoefficients ────────────────────────────────────────────────────

static void test_from_coefficients() {
    // 5 control points, all same value → constant spline
    const int n = 5;
    std::vector<double> coeffs(n, 7.0);
    UniformCubicBSpline sp(1);
    sp.initFromCoefficients(0.0, 0.5, coeffs.data(), n);
    assert(sp.isValid());

    double val;
    bool ok = sp.eval(sp.tMin(), &val);
    assert(ok);
    assert(near(val, 7.0, 1e-3));
    std::puts("  initFromCoefficients     OK");
}

// ── Main ────────────────────────────────────────────────────────────────────

int main() {
    std::puts("BSpline:");
    test_construction();
    test_constant_signal();
    test_linear_signal();
    test_linear_derivative();
    test_3d_spline();
    test_out_of_range();
    test_second_derivative();
    test_from_coefficients();
    std::puts("BSpline: ALL TESTS PASSED");
    return 0;
}
