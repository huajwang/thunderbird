// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Evaluation Framework: Trajectory Alignment Impl
// ─────────────────────────────────────────────────────────────────────────────
//
// Horn's closed-form unit-quaternion method for SE(3)/Sim(3) alignment.
//
// Algorithm outline:
//   1. Compute centroids of source and target point sets.
//   2. Build the 3×3 cross-covariance matrix H = Σ (src_i - c_s)(tgt_i - c_t)^T.
//   3. Construct Horn's 4×4 symmetric matrix N from H.
//   4. Find the eigenvector of N corresponding to the largest eigenvalue
//      using Jacobi iteration — this is the optimal rotation quaternion.
//   5. Optionally recover scale (Umeyama): s = Σ||tgt_centered||² / Σ||src_centered||².
//   6. Compute translation: t = centroid_tgt - s * R * centroid_src.
//   7. If outlier rejection is enabled, iterate: mark inliers by thresholding
//      point-wise error, re-solve on inliers only, repeat until convergence.
//
// ─────────────────────────────────────────────────────────────────────────────

#include "eval/trajectory_alignment.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <numeric>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace eval {

using Vec3  = std::array<double,3>;
using Quat4 = std::array<double,4>;

// ═════════════════════════════════════════════════════════════════════════════
//  Local helpers
// ═════════════════════════════════════════════════════════════════════════════

static Vec3 centroid(const std::vector<PointPair>& pairs, bool target) {
    Vec3 c{};
    for (const auto& p : pairs) {
        const auto& v = target ? p.target : p.source;
        c[0] += v[0]; c[1] += v[1]; c[2] += v[2];
    }
    double n = static_cast<double>(pairs.size());
    c[0] /= n; c[1] /= n; c[2] /= n;
    return c;
}

static double vec_dot(const Vec3& a, const Vec3& b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static double vec_norm2(const Vec3& v) {
    return v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
}

static double vec_norm(const Vec3& v) {
    return std::sqrt(vec_norm2(v));
}

static Vec3 vec_sub(const Vec3& a, const Vec3& b) {
    return {a[0]-b[0], a[1]-b[1], a[2]-b[2]};
}

// ═════════════════════════════════════════════════════════════════════════════
//  Horn's N matrix construction
// ═════════════════════════════════════════════════════════════════════════════

std::array<double,16> horn_detail::buildN(
    const std::vector<PointPair>& pairs,
    const Vec3& cs, const Vec3& ct)
{
    // Accumulate 3×3 cross-covariance: H = Σ (s_i - cs)(t_i - ct)^T
    // H is stored as H[row][col] = Sxx, Sxy, Sxz, ...
    double Sxx=0, Sxy=0, Sxz=0;
    double Syx=0, Syy=0, Syz=0;
    double Szx=0, Szy=0, Szz=0;

    for (const auto& p : pairs) {
        double sx = p.source[0]-cs[0], sy = p.source[1]-cs[1], sz = p.source[2]-cs[2];
        double tx = p.target[0]-ct[0], ty = p.target[1]-ct[1], tz = p.target[2]-ct[2];
        Sxx += sx*tx; Sxy += sx*ty; Sxz += sx*tz;
        Syx += sy*tx; Syy += sy*ty; Syz += sy*tz;
        Szx += sz*tx; Szy += sz*ty; Szz += sz*tz;
    }

    // Horn's 4×4 symmetric N matrix (row-major):
    //
    //     N = [ Sxx+Syy+Szz,  Syz-Szy,      Szx-Sxz,      Sxy-Syx     ]
    //         [ Syz-Szy,      Sxx-Syy-Szz,   Sxy+Syx,      Szx+Sxz     ]
    //         [ Szx-Sxz,      Sxy+Syx,      -Sxx+Syy-Szz,  Syz+Szy     ]
    //         [ Sxy-Syx,      Szx+Sxz,       Syz+Szy,     -Sxx-Syy+Szz ]
    //
    std::array<double,16> N{};
    N[ 0] =  Sxx+Syy+Szz;  N[ 1] =  Syz-Szy;      N[ 2] =  Szx-Sxz;      N[ 3] =  Sxy-Syx;
    N[ 4] =  Syz-Szy;      N[ 5] =  Sxx-Syy-Szz;  N[ 6] =  Sxy+Syx;      N[ 7] =  Szx+Sxz;
    N[ 8] =  Szx-Sxz;      N[ 9] =  Sxy+Syx;      N[10] = -Sxx+Syy-Szz;  N[11] =  Syz+Szy;
    N[12] =  Sxy-Syx;      N[13] =  Szx+Sxz;      N[14] =  Syz+Szy;      N[15] = -Sxx-Syy+Szz;

    return N;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Analytical eigenvalue solver for 4×4 symmetric matrix
// ═════════════════════════════════════════════════════════════════════════════
//
// Closed-form O(1) solver for the maximum eigenvector of a 4×4 symmetric
// matrix (Horn's N matrix).  Replaces the iterative Jacobi scheme.
//
// Algorithm:
//   1. Compute the characteristic polynomial λ⁴ − c₃λ³ + c₂λ² − c₁λ + c₀ = 0.
//   2. Reduce to a depressed quartic u⁴ + pu² + qu + r = 0 (Ferrari).
//   3. Solve the resolvent cubic via Vieta's trigonometric formula
//      (all roots real for symmetric matrices).
//   4. Factor the quartic into two quadratics, solve both.
//   5. Pick the largest eigenvalue λ_max, polish with Newton–Raphson.
//   6. Recover the eigenvector from the adjugate of (A − λ_max·I).
//
// Complexity: O(1) constant time, zero iteration.
// Dependencies: <cmath> only.

Quat4 horn_detail::maxEigenvector4x4(const std::array<double,16>& N_in) {
    constexpr double EPS = 1e-14;

    // ── 3×3 determinant helper ──────────────────────────────────────────
    auto det3 = [](double a11, double a12, double a13,
                   double a21, double a22, double a23,
                   double a31, double a32, double a33) -> double {
        return a11*(a22*a33 - a23*a32)
             - a12*(a21*a33 - a23*a31)
             + a13*(a21*a32 - a22*a31);
    };

    // 3×3 determinant for a symmetric matrix [a b c; b d e; c e f].
    auto det3sym = [](double a, double b, double c,
                      double d, double e, double f) -> double {
        return a*(d*f - e*e) - b*(b*f - c*e) + c*(b*e - c*d);
    };

    // ── Extract upper triangle (matrix is symmetric) ────────────────────
    const double a00 = N_in[0],  a01 = N_in[1],  a02 = N_in[2],  a03 = N_in[3];
    const double a11 = N_in[5],  a12 = N_in[6],  a13 = N_in[7];
    const double a22 = N_in[10], a23 = N_in[11];
    const double a33 = N_in[15];

    // ════════════════════════════════════════════════════════════════════
    //  Step 1: Characteristic polynomial  λ⁴ − c₃λ³ + c₂λ² − c₁λ + c₀ = 0
    // ════════════════════════════════════════════════════════════════════

    // c₃ = tr(A)
    const double c3 = a00 + a11 + a22 + a33;

    // c₂ = Σ 2×2 principal minors
    const double c2 = (a00*a11 - a01*a01) + (a00*a22 - a02*a02)
                    + (a00*a33 - a03*a03) + (a11*a22 - a12*a12)
                    + (a11*a33 - a13*a13) + (a22*a33 - a23*a23);

    // c₁ = Σ 3×3 principal minors (cofactors of each diagonal element)
    const double c1 = det3sym(a11, a12, a13, a22, a23, a33)   // M₀₀
                    + det3sym(a00, a02, a03, a22, a23, a33)    // M₁₁
                    + det3sym(a00, a01, a03, a11, a13, a33)    // M₂₂
                    + det3sym(a00, a01, a02, a11, a12, a22);   // M₃₃

    // c₀ = det(A) via cofactor expansion along row 0
    const double c0 =
          a00 * det3(a11,a12,a13, a12,a22,a23, a13,a23,a33)
        - a01 * det3(a01,a12,a13, a02,a22,a23, a03,a23,a33)
        + a02 * det3(a01,a11,a13, a02,a12,a23, a03,a13,a33)
        - a03 * det3(a01,a11,a12, a02,a12,a22, a03,a13,a23);

    // ════════════════════════════════════════════════════════════════════
    //  Step 2: Depressed quartic  u⁴ + pu² + qu + r = 0  via  λ = u + c₃/4
    // ════════════════════════════════════════════════════════════════════

    // Standard monic form: x⁴ + ax³ + bx² + cx + d = 0
    //   a = -c₃,  b = c₂,  c = -c₁,  d = c₀
    const double qa = -c3;
    const double qa2 = qa * qa;
    const double p  =  c2 - 3.0 * qa2 / 8.0;
    // q = a³/8 − ab/2 + c = (−c₃)³/8 − (−c₃)(c₂)/2 + (−c₁)
    const double qq =  qa * qa2 / 8.0 - qa * c2 / 2.0 - c1;
    const double rr = -3.0 * qa2 * qa2 / 256.0 + qa2 * c2 / 16.0 + qa * c1 / 4.0 + c0;
    const double shift = c3 / 4.0;   //  λ = u + c₃/4

    // ════════════════════════════════════════════════════════════════════
    //  Step 3–4: Solve the quartic
    // ════════════════════════════════════════════════════════════════════

    double roots[4];

    if (std::abs(qq) < EPS) {
        // ── Biquadratic u⁴ + pu² + r = 0  →  w² + pw + r = 0 ──────────
        double disc = p * p - 4.0 * rr;
        if (disc < 0.0) disc = 0.0;
        const double sd = std::sqrt(disc);
        const double w1 = (-p + sd) / 2.0;
        const double w2 = (-p - sd) / 2.0;
        const double sw1 = std::sqrt(std::max(w1, 0.0));
        const double sw2 = std::sqrt(std::max(w2, 0.0));
        roots[0] =  sw1 + shift;
        roots[1] = -sw1 + shift;
        roots[2] =  sw2 + shift;
        roots[3] = -sw2 + shift;
    } else {
        // ── Ferrari's resolvent cubic ───────────────────────────────────
        //
        //   8m³ + 8pm² + (2p²−8r)m − q² = 0
        //   → m³ + p·m² + (p²−4r)/4·m − q²/8 = 0
        //
        // Depress via m = w − p/3:
        //   w³ + hw + g = 0
        //
        const double rc_b = (p * p - 4.0 * rr) / 4.0;
        const double rc_c = -qq * qq / 8.0;
        const double h = rc_b - p * p / 3.0;        // = (−p² − 12r)/12
        const double g = 2.0*p*p*p/27.0 - p*rc_b/3.0 + rc_c;

        double m;
        if (std::abs(h) < EPS) {
            // Degenerate cubic: w³ ≈ −g
            m = std::cbrt(-g) - p / 3.0;
        } else if (h < 0.0) {
            // ── Vieta's trigonometric formula (3 real roots) ─────────────
            const double neg_h = -h;
            const double k = 2.0 * std::sqrt(neg_h / 3.0);
            double cos_arg = -3.0 * g * std::sqrt(3.0)
                           / (2.0 * neg_h * std::sqrt(neg_h));
            if (cos_arg >  1.0) cos_arg =  1.0;
            if (cos_arg < -1.0) cos_arg = -1.0;
            const double phi = std::acos(cos_arg) / 3.0;

            const double w0 = k * std::cos(phi);
            const double w1 = k * std::cos(phi - 2.0 * M_PI / 3.0);
            const double w2 = k * std::cos(phi - 4.0 * M_PI / 3.0);

            // Pick the largest root (guaranteed positive for all-real quartic).
            m = std::max({w0, w1, w2}) - p / 3.0;
        } else {
            // ── Cardano fallback (one real root) ────────────────────────
            const double D = g*g/4.0 + h*h*h/27.0;
            const double sqrtD = std::sqrt(std::max(D, 0.0));
            m = std::cbrt(-g/2.0 + sqrtD) + std::cbrt(-g/2.0 - sqrtD) - p / 3.0;
        }

        // Ensure m > 0 (required for s = √(2m)).
        if (m < EPS) m = EPS;

        // ── Factor into two quadratics ──────────────────────────────────
        //   (u² − su + α)(u² + su + β) = 0
        //   where s = √(2m), α = p/2 + m + q/(2s), β = p/2 + m − q/(2s)
        const double s  = std::sqrt(2.0 * m);
        const double al = p / 2.0 + m + qq / (2.0 * s);
        const double be = p / 2.0 + m - qq / (2.0 * s);

        double d1 = s * s - 4.0 * al;
        if (d1 < 0.0) d1 = 0.0;
        const double sd1 = std::sqrt(d1);
        roots[0] = ( s + sd1) / 2.0 + shift;
        roots[1] = ( s - sd1) / 2.0 + shift;

        double d2 = s * s - 4.0 * be;
        if (d2 < 0.0) d2 = 0.0;
        const double sd2 = std::sqrt(d2);
        roots[2] = (-s + sd2) / 2.0 + shift;
        roots[3] = (-s - sd2) / 2.0 + shift;
    }

    // ════════════════════════════════════════════════════════════════════
    //  Step 5: Largest eigenvalue + Newton–Raphson polish
    // ════════════════════════════════════════════════════════════════════

    double lam = roots[0];
    for (int i = 1; i < 4; ++i)
        if (roots[i] > lam) lam = roots[i];

    // Two Newton iterations on f(λ) = λ⁴ − c₃λ³ + c₂λ² − c₁λ + c₀.
    for (int i = 0; i < 2; ++i) {
        const double l2 = lam * lam;
        const double f  = l2*l2 - c3*l2*lam + c2*l2 - c1*lam + c0;
        const double fp = 4.0*l2*lam - 3.0*c3*l2 + 2.0*c2*lam - c1;
        if (std::abs(fp) > EPS) lam -= f / fp;
    }

    // ════════════════════════════════════════════════════════════════════
    //  Step 6: Eigenvector via adjugate of B = (A − λ_max·I)
    // ════════════════════════════════════════════════════════════════════
    //
    // B is rank 3.  Any non-zero column of adj(B) is the null vector.
    // We try all 4 columns and pick the one with the largest norm² for
    // numerical stability.

    const double b00 = a00 - lam, b01 = a01, b02 = a02, b03 = a03;
    const double b11 = a11 - lam, b12 = a12, b13 = a13;
    const double b22 = a22 - lam, b23 = a23;
    const double b33 = a33 - lam;

    Quat4 ev{};
    double best_n2 = 0.0;

    // Column 0: adj(B)_{j,0} = (−1)^j · M_{0j}
    {
        double v0 =  det3(b11,b12,b13, b12,b22,b23, b13,b23,b33);
        double v1 = -det3(b01,b12,b13, b02,b22,b23, b03,b23,b33);
        double v2 =  det3(b01,b11,b13, b02,b12,b23, b03,b13,b33);
        double v3 = -det3(b01,b11,b12, b02,b12,b22, b03,b13,b23);
        double n2 = v0*v0 + v1*v1 + v2*v2 + v3*v3;
        if (n2 > best_n2) { best_n2 = n2; ev = {v0, v1, v2, v3}; }
    }
    // Column 1: adj(B)_{j,1} = (−1)^{j+1} · M_{1j}
    {
        double v0 = -det3(b01,b02,b03, b12,b22,b23, b13,b23,b33);
        double v1 =  det3(b00,b02,b03, b02,b22,b23, b03,b23,b33);
        double v2 = -det3(b00,b01,b03, b02,b12,b23, b03,b13,b33);
        double v3 =  det3(b00,b01,b02, b02,b12,b22, b03,b13,b23);
        double n2 = v0*v0 + v1*v1 + v2*v2 + v3*v3;
        if (n2 > best_n2) { best_n2 = n2; ev = {v0, v1, v2, v3}; }
    }
    // Column 2: adj(B)_{j,2} = (−1)^j · M_{2j}
    {
        double v0 =  det3(b01,b02,b03, b11,b12,b13, b13,b23,b33);
        double v1 = -det3(b00,b02,b03, b01,b12,b13, b03,b23,b33);
        double v2 =  det3(b00,b01,b03, b01,b11,b13, b03,b13,b33);
        double v3 = -det3(b00,b01,b02, b01,b11,b12, b03,b13,b23);
        double n2 = v0*v0 + v1*v1 + v2*v2 + v3*v3;
        if (n2 > best_n2) { best_n2 = n2; ev = {v0, v1, v2, v3}; }
    }
    // Column 3: adj(B)_{j,3} = (−1)^{j+1} · M_{3j}
    {
        double v0 = -det3(b01,b02,b03, b11,b12,b13, b12,b22,b23);
        double v1 =  det3(b00,b02,b03, b01,b12,b13, b02,b22,b23);
        double v2 = -det3(b00,b01,b03, b01,b11,b13, b02,b12,b23);
        double v3 =  det3(b00,b01,b02, b01,b11,b12, b02,b12,b22);
        double n2 = v0*v0 + v1*v1 + v2*v2 + v3*v3;
        if (n2 > best_n2) { best_n2 = n2; ev = {v0, v1, v2, v3}; }
    }

    // Normalise.
    double n = std::sqrt(best_n2);
    if (n > 0.0) { ev[0]/=n; ev[1]/=n; ev[2]/=n; ev[3]/=n; }

    // Ensure w > 0 for canonical form.
    if (ev[0] < 0.0) { ev[0]=-ev[0]; ev[1]=-ev[1]; ev[2]=-ev[2]; ev[3]=-ev[3]; }

    return ev;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Rotate vector by quaternion
// ═════════════════════════════════════════════════════════════════════════════

Vec3 horn_detail::rotateByQuat(const Quat4& q, const Vec3& v) {
    // q ⊗ [0, v] ⊗ q*   (Hamilton convention, q = [w,x,y,z])
    double w = q[0], x = q[1], y = q[2], z = q[3];

    // Efficient formula: v' = v + 2w(u×v) + 2(u×(u×v))  where u = [x,y,z]
    double ux = y*v[2] - z*v[1];
    double uy = z*v[0] - x*v[2];
    double uz = x*v[1] - y*v[0];

    return {
        v[0] + 2.0*(w*ux + y*uz - z*uy),
        v[1] + 2.0*(w*uy + z*ux - x*uz),
        v[2] + 2.0*(w*uz + x*uy - y*ux)
    };
}

// ═════════════════════════════════════════════════════════════════════════════
//  RMSE computation
// ═════════════════════════════════════════════════════════════════════════════

double horn_detail::computeRmse(const std::vector<PointPair>& pairs,
                                 const Quat4& quat,
                                 const Vec3& trans,
                                 double scale)
{
    if (pairs.empty()) return 0.0;
    double sum_sq = 0.0;
    for (const auto& p : pairs) {
        auto rot = rotateByQuat(quat, p.source);
        Vec3 aligned = {
            scale * rot[0] + trans[0],
            scale * rot[1] + trans[1],
            scale * rot[2] + trans[2]
        };
        sum_sq += vec_norm2(vec_sub(p.target, aligned));
    }
    return std::sqrt(sum_sq / static_cast<double>(pairs.size()));
}

// ═════════════════════════════════════════════════════════════════════════════
//  Core alignment: Horn + optional scale + optional outlier rejection
// ═════════════════════════════════════════════════════════════════════════════

/// Solve alignment on a given subset of pairs (no outlier rejection).
static AlignmentResult solveHorn(const std::vector<PointPair>& pairs,
                                  bool estimate_scale) {
    AlignmentResult result;
    result.total_pairs  = pairs.size();
    result.inlier_count = pairs.size();

    if (pairs.size() < 3) {
        std::cerr << "[align] need ≥3 pairs, got " << pairs.size() << "\n";
        return result;
    }

    // 1. Centroids.
    Vec3 cs = centroid(pairs, false);
    Vec3 ct = centroid(pairs, true);

    // 2. Build N matrix from cross-covariance.
    auto N = horn_detail::buildN(pairs, cs, ct);

    // 3. Find optimal rotation quaternion.
    result.rotation = horn_detail::maxEigenvector4x4(N);

    // 4. Scale (Umeyama).
    if (estimate_scale) {
        double num = 0.0, den = 0.0;
        for (const auto& p : pairs) {
            auto sc = vec_sub(p.source, cs);
            auto tc = vec_sub(p.target, ct);
            num += vec_norm2(tc);
            den += vec_norm2(sc);
        }
        result.scale = (den > 0.0) ? std::sqrt(num / den) : 1.0;
    } else {
        result.scale = 1.0;
    }

    // 5. Translation.
    auto r_cs = horn_detail::rotateByQuat(result.rotation, cs);
    result.translation = {
        ct[0] - result.scale * r_cs[0],
        ct[1] - result.scale * r_cs[1],
        ct[2] - result.scale * r_cs[2]
    };

    // 6. RMSE after alignment.
    result.rmse_after = horn_detail::computeRmse(
        pairs, result.rotation, result.translation, result.scale);

    return result;
}

AlignmentResult alignTrajectories(const std::vector<PointPair>& pairs,
                                   AlignmentConfig cfg)
{
    if (pairs.size() < 3) {
        AlignmentResult r;
        r.total_pairs = pairs.size();
        std::cerr << "[align] need ≥3 pairs for alignment\n";
        return r;
    }

    // Pre-alignment RMSE (identity transform).
    double rmse_before = 0.0;
    {
        double sum_sq = 0.0;
        for (const auto& p : pairs) {
            sum_sq += vec_norm2(vec_sub(p.target, p.source));
        }
        rmse_before = std::sqrt(sum_sq / static_cast<double>(pairs.size()));
    }

    if (!cfg.outlier_rejection) {
        // ── Simple case: align all pairs ────────────────────────────────
        auto r = solveHorn(pairs, cfg.estimate_scale);
        r.rmse_before = rmse_before;
        r.converged   = true;
        r.iterations  = 1;
        return r;
    }

    // ── Iterative outlier rejection ─────────────────────────────────────
    //
    // 1.  Solve alignment on all pairs.
    // 2.  Compute per-pair error after alignment.
    // 3.  Mark outliers: error > mean + threshold_sigma * stddev.
    // 4.  Remove outliers, re-solve.
    // 5.  Repeat until inlier set stabilises or max_iters reached.

    std::vector<bool> inlier(pairs.size(), true);
    AlignmentResult best_result;
    best_result.total_pairs  = pairs.size();
    best_result.rmse_before  = rmse_before;

    for (size_t iter = 0; iter < cfg.max_outlier_iters; ++iter) {
        // Collect inlier pairs.
        std::vector<PointPair> inlier_pairs;
        inlier_pairs.reserve(pairs.size());
        for (size_t i = 0; i < pairs.size(); ++i) {
            if (inlier[i]) inlier_pairs.push_back(pairs[i]);
        }

        if (inlier_pairs.size() < 3) {
            std::cerr << "[align] outlier rejection left <3 inliers\n";
            break;
        }

        // Check minimum inlier ratio.
        double ratio = static_cast<double>(inlier_pairs.size()) /
                       static_cast<double>(pairs.size());
        if (ratio < cfg.min_inlier_ratio) {
            std::cerr << "[align] inlier ratio " << ratio
                      << " below minimum " << cfg.min_inlier_ratio << "\n";
            break;
        }

        auto r = solveHorn(inlier_pairs, cfg.estimate_scale);
        r.total_pairs  = pairs.size();
        r.rmse_before  = rmse_before;
        r.iterations   = iter + 1;

        // Compute per-pair errors (on ALL pairs, including current outliers).
        std::vector<double> errors(pairs.size());
        double sum_err = 0.0, sum_err2 = 0.0;
        for (size_t i = 0; i < pairs.size(); ++i) {
            auto rot = horn_detail::rotateByQuat(r.rotation, pairs[i].source);
            Vec3 aligned = {
                r.scale * rot[0] + r.translation[0],
                r.scale * rot[1] + r.translation[1],
                r.scale * rot[2] + r.translation[2]
            };
            errors[i] = vec_norm(vec_sub(pairs[i].target, aligned));
            if (inlier[i]) {
                sum_err  += errors[i];
                sum_err2 += errors[i] * errors[i];
            }
        }

        double n_inlier = static_cast<double>(inlier_pairs.size());
        double mean_err = sum_err / n_inlier;
        double var_err  = (sum_err2 / n_inlier) - mean_err * mean_err;
        double std_err  = std::sqrt(std::max(var_err, 0.0));
        double threshold = mean_err + cfg.outlier_threshold_sigma * std_err;

        // Update inlier mask.
        size_t new_outliers = 0;
        std::vector<bool> new_inlier(pairs.size());
        for (size_t i = 0; i < pairs.size(); ++i) {
            new_inlier[i] = (errors[i] <= threshold);
            if (!new_inlier[i] && inlier[i]) ++new_outliers;
        }

        r.inlier_count  = std::count(new_inlier.begin(), new_inlier.end(), true);
        r.outlier_count  = pairs.size() - r.inlier_count;
        best_result = r;

        // Check convergence: no new outliers removed.
        if (new_outliers == 0) {
            best_result.converged = true;
            break;
        }

        inlier = new_inlier;
    }

    // Final RMSE on ALL pairs (not just inliers).
    best_result.rmse_after = horn_detail::computeRmse(
        pairs, best_result.rotation, best_result.translation, best_result.scale);

    return best_result;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Apply alignment
// ═════════════════════════════════════════════════════════════════════════════

std::vector<Vec3> applyAlignment(const std::vector<Vec3>& source,
                                  const AlignmentResult& a) {
    std::vector<Vec3> result;
    result.reserve(source.size());
    for (const auto& pt : source) {
        result.push_back(applyAlignment(pt, a));
    }
    return result;
}

Vec3 applyAlignment(const Vec3& point, const AlignmentResult& a) {
    auto rot = horn_detail::rotateByQuat(a.rotation, point);
    return {
        a.scale * rot[0] + a.translation[0],
        a.scale * rot[1] + a.translation[1],
        a.scale * rot[2] + a.translation[2]
    };
}

} // namespace eval
