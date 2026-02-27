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
//  Jacobi eigenvalue solver for 4×4 symmetric matrix
// ═════════════════════════════════════════════════════════════════════════════
//
// Finds ALL eigenvalues/eigenvectors, then returns the eigenvector for the
// largest eigenvalue.  Jacobi is overkill for 4×4 but is bullet-proof and
// requires zero external dependencies.

Quat4 horn_detail::maxEigenvector4x4(const std::array<double,16>& N_in) {
    constexpr int SZ = 4;
    constexpr int MAX_ITER = 100;
    constexpr double EPS = 1e-15;

    // Working copy of N (symmetric, row-major).
    double A[SZ][SZ];
    for (int i = 0; i < SZ; ++i)
        for (int j = 0; j < SZ; ++j)
            A[i][j] = N_in[i*SZ + j];

    // Eigenvector matrix (starts as identity).
    double V[SZ][SZ] = {};
    for (int i = 0; i < SZ; ++i) V[i][i] = 1.0;

    for (int iter = 0; iter < MAX_ITER; ++iter) {
        // Find the largest off-diagonal element.
        int p = 0, q = 1;
        double max_off = 0.0;
        for (int i = 0; i < SZ; ++i) {
            for (int j = i+1; j < SZ; ++j) {
                if (std::abs(A[i][j]) > max_off) {
                    max_off = std::abs(A[i][j]);
                    p = i; q = j;
                }
            }
        }
        if (max_off < EPS) break;  // converged

        // Compute Jacobi rotation.
        double app = A[p][p], aqq = A[q][q], apq = A[p][q];
        double theta;
        if (std::abs(app - aqq) < EPS) {
            theta = M_PI / 4.0;
        } else {
            theta = 0.5 * std::atan2(2.0 * apq, app - aqq);
        }
        double c = std::cos(theta), s = std::sin(theta);

        // Apply rotation to A: A' = G^T A G.
        // Update rows/cols p and q of A.
        double new_pp = c*c*app + 2*s*c*apq + s*s*aqq;
        double new_qq = s*s*app - 2*s*c*apq + c*c*aqq;
        A[p][p] = new_pp;
        A[q][q] = new_qq;
        A[p][q] = A[q][p] = 0.0;  // zeroed by design

        for (int r = 0; r < SZ; ++r) {
            if (r == p || r == q) continue;
            double arp = A[r][p], arq = A[r][q];
            A[r][p] = A[p][r] = c*arp + s*arq;
            A[r][q] = A[q][r] = -s*arp + c*arq;
        }

        // Accumulate eigenvectors.
        for (int r = 0; r < SZ; ++r) {
            double vp = V[r][p], vq = V[r][q];
            V[r][p] = c*vp + s*vq;
            V[r][q] = -s*vp + c*vq;
        }
    }

    // Find index of largest eigenvalue.
    int best = 0;
    for (int i = 1; i < SZ; ++i) {
        if (A[i][i] > A[best][best]) best = i;
    }

    // Extract corresponding eigenvector and normalise.
    Quat4 q = {V[0][best], V[1][best], V[2][best], V[3][best]};
    double n = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n > 0.0) { q[0]/=n; q[1]/=n; q[2]/=n; q[3]/=n; }

    // Ensure w > 0 for canonical form.
    if (q[0] < 0.0) { q[0]=-q[0]; q[1]=-q[1]; q[2]=-q[2]; q[3]=-q[3]; }

    return q;
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
