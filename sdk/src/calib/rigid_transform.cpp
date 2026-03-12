// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Rigid Transform Solver Implementation
// ─────────────────────────────────────────────────────────────────────────────
//
// Horn's SVD method with a self-contained 3×3 SVD (Jacobi rotations).
// No external dependencies — uses only <cmath> and <cstring>.
//
// ─────────────────────────────────────────────────────────────────────────────
#include "rigid_transform.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numbers>
#include <random>

namespace thunderbird::calib {
namespace {

// ─── 3×3 Matrix helpers (row-major) ────────────────────────────────────────

using Mat3 = double[9];  // row-major 3×3

inline double& m3(double* M, int r, int c) { return M[r * 3 + c]; }
inline double  m3(const double* M, int r, int c) { return M[r * 3 + c]; }

void mat3_identity(double* M) {
    std::memset(M, 0, 9 * sizeof(double));
    M[0] = M[4] = M[8] = 1.0;
}

void mat3_transpose(const double* A, double* At) {
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            m3(At, c, r) = m3(A, r, c);
}

void mat3_mul(const double* A, const double* B, double* C) {
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) {
            double s = 0.0;
            for (int k = 0; k < 3; ++k)
                s += m3(A, r, k) * m3(B, k, c);
            m3(C, r, c) = s;
        }
}

double mat3_det(const double* M) {
    return m3(M, 0, 0) * (m3(M, 1, 1) * m3(M, 2, 2) - m3(M, 1, 2) * m3(M, 2, 1))
         - m3(M, 0, 1) * (m3(M, 1, 0) * m3(M, 2, 2) - m3(M, 1, 2) * m3(M, 2, 0))
         + m3(M, 0, 2) * (m3(M, 1, 0) * m3(M, 2, 1) - m3(M, 1, 1) * m3(M, 2, 0));
}

// ─── Jacobi SVD for 3×3 ───────────────────────────────────────────────────
//
// Decomposes A = U Σ V^T using Jacobi rotations on A^T A.
//
// Algorithm:
//   1. Compute S = A^T A  (3×3 symmetric positive semi-definite)
//   2. Diagonalize S via Jacobi rotations → S = V D V^T
//   3. Singular values = sqrt(D)
//   4. U = A V Σ^{-1}  (for non-zero singular values)
//   5. Handle sign/ordering

void jacobi_svd3(const double* A, double* U, double sigma[3], double* V) {
    // Step 1: S = A^T A
    double At[9], S[9];
    mat3_transpose(A, At);
    mat3_mul(At, A, S);

    // Step 2: Jacobi eigendecomposition of S
    mat3_identity(V);
    double D[9];
    std::memcpy(D, S, 9 * sizeof(double));

    // Jacobi iterations
    constexpr int max_iter = 50;
    constexpr double eps = 1e-15;

    for (int iter = 0; iter < max_iter; ++iter) {
        // Find largest off-diagonal element
        double max_off = 0.0;
        int p = 0, q = 1;
        for (int r = 0; r < 3; ++r)
            for (int c = r + 1; c < 3; ++c)
                if (std::fabs(m3(D, r, c)) > max_off) {
                    max_off = std::fabs(m3(D, r, c));
                    p = r; q = c;
                }

        if (max_off < eps) break;

        // Compute Jacobi rotation angle
        double app = m3(D, p, p);
        double aqq = m3(D, q, q);
        double apq = m3(D, p, q);

        double theta;
        if (std::fabs(app - aqq) < eps) {
            theta = (apq > 0.0) ? -std::numbers::pi / 4.0
                                :  std::numbers::pi / 4.0;
        } else {
            theta = 0.5 * std::atan2(-2.0 * apq, app - aqq);
        }

        double c = std::cos(theta);
        double s = std::sin(theta);

        // Apply Givens rotation: D <- G^T D G
        // This is the standard Jacobi update for symmetric matrices
        double G[9];
        mat3_identity(G);
        m3(G, p, p) = c;  m3(G, p, q) = s;
        m3(G, q, p) = -s; m3(G, q, q) = c;

        double Gt[9], tmp[9], D2[9];
        mat3_transpose(G, Gt);
        mat3_mul(Gt, D, tmp);
        mat3_mul(tmp, G, D2);
        std::memcpy(D, D2, 9 * sizeof(double));

        // Accumulate V: V = V * G
        double V2[9];
        mat3_mul(V, G, V2);
        std::memcpy(V, V2, 9 * sizeof(double));
    }

    // Eigenvalues are on diagonal of D (may be negative due to numerical noise)
    double eig[3] = { m3(D, 0, 0), m3(D, 1, 1), m3(D, 2, 2) };

    // Sort eigenvalues descending, rearrange V columns accordingly
    int idx[3] = {0, 1, 2};
    if (eig[idx[0]] < eig[idx[1]]) std::swap(idx[0], idx[1]);
    if (eig[idx[0]] < eig[idx[2]]) std::swap(idx[0], idx[2]);
    if (eig[idx[1]] < eig[idx[2]]) std::swap(idx[1], idx[2]);

    double V_sorted[9];
    for (int c = 0; c < 3; ++c) {
        int src = idx[c];
        for (int r = 0; r < 3; ++r)
            m3(V_sorted, r, c) = m3(V, r, src);
        sigma[c] = std::sqrt(std::max(0.0, eig[src]));
    }
    std::memcpy(V, V_sorted, 9 * sizeof(double));

    // Step 4: U = A V Σ^{-1}
    double AV[9];
    mat3_mul(A, V, AV);

    for (int c = 0; c < 3; ++c) {
        if (sigma[c] > eps) {
            double inv_s = 1.0 / sigma[c];
            for (int r = 0; r < 3; ++r)
                m3(U, r, c) = m3(AV, r, c) * inv_s;
        } else {
            // Zero singular value: set column to zero (will be fixed below)
            for (int r = 0; r < 3; ++r)
                m3(U, r, c) = 0.0;
        }
    }

    // Ensure U is a proper rotation matrix (handle rank-deficient case)
    // If det(U) < 0, negate the column corresponding to smallest singular value
    if (mat3_det(U) < 0.0) {
        for (int r = 0; r < 3; ++r)
            m3(U, r, 2) = -m3(U, r, 2);
    }
}

// ─── Rotation matrix → quaternion [w,x,y,z] ───────────────────────────────

void rotMatToQuat(const double* R, double q[4]) {
    double tr = R[0] + R[4] + R[8];
    if (tr > 0.0) {
        double s = 0.5 / std::sqrt(tr + 1.0);
        q[0] = 0.25 / s;
        q[1] = (m3(R, 2, 1) - m3(R, 1, 2)) * s;
        q[2] = (m3(R, 0, 2) - m3(R, 2, 0)) * s;
        q[3] = (m3(R, 1, 0) - m3(R, 0, 1)) * s;
    } else if (R[0] > R[4] && R[0] > R[8]) {
        double s = 2.0 * std::sqrt(1.0 + R[0] - R[4] - R[8]);
        q[0] = (m3(R, 2, 1) - m3(R, 1, 2)) / s;
        q[1] = 0.25 * s;
        q[2] = (m3(R, 0, 1) + m3(R, 1, 0)) / s;
        q[3] = (m3(R, 0, 2) + m3(R, 2, 0)) / s;
    } else if (R[4] > R[8]) {
        double s = 2.0 * std::sqrt(1.0 + R[4] - R[0] - R[8]);
        q[0] = (m3(R, 0, 2) - m3(R, 2, 0)) / s;
        q[1] = (m3(R, 0, 1) + m3(R, 1, 0)) / s;
        q[2] = 0.25 * s;
        q[3] = (m3(R, 1, 2) + m3(R, 2, 1)) / s;
    } else {
        double s = 2.0 * std::sqrt(1.0 + R[8] - R[0] - R[4]);
        q[0] = (m3(R, 1, 0) - m3(R, 0, 1)) / s;
        q[1] = (m3(R, 0, 2) + m3(R, 2, 0)) / s;
        q[2] = (m3(R, 1, 2) + m3(R, 2, 1)) / s;
        q[3] = 0.25 * s;
    }
    // Normalize
    double n = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n > 0.0) { q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n; }
    // Positive w convention
    if (q[0] < 0.0) { q[0] = -q[0]; q[1] = -q[1]; q[2] = -q[2]; q[3] = -q[3]; }
}

// ─── Apply rotation ───────────────────────────────────────────────────────

void mat3_vec(const double* R, const double* v, double* out) {
    for (int r = 0; r < 3; ++r) {
        out[r] = 0.0;
        for (int c = 0; c < 3; ++c)
            out[r] += m3(R, r, c) * v[c];
    }
}

}  // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────

RigidTransformResult solveRigidTransform(const double* source,
                                         const double* target,
                                         int n_points) {
    RigidTransformResult result;
    if (!source || !target || n_points < 3) return result;

    // Step 1: Compute centroids
    double c_src[3] = {}, c_tgt[3] = {};
    for (int i = 0; i < n_points; ++i) {
        for (int j = 0; j < 3; ++j) {
            c_src[j] += source[i * 3 + j];
            c_tgt[j] += target[i * 3 + j];
        }
    }
    double inv_n = 1.0 / static_cast<double>(n_points);
    for (int j = 0; j < 3; ++j) {
        c_src[j] *= inv_n;
        c_tgt[j] *= inv_n;
    }

    // Step 2: Cross-covariance H = Σ (p_i - c_src) (q_i - c_tgt)^T
    double H[9] = {};
    for (int i = 0; i < n_points; ++i) {
        double ps[3], pt[3];
        for (int j = 0; j < 3; ++j) {
            ps[j] = source[i * 3 + j] - c_src[j];
            pt[j] = target[i * 3 + j] - c_tgt[j];
        }
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                H[r * 3 + c] += ps[r] * pt[c];
    }

    // Step 3: SVD of H → U Σ V^T
    double U[9], sigma[3], V[9];
    jacobi_svd3(H, U, sigma, V);

    // Step 4: R = V U^T, with reflection correction
    double Ut[9];
    mat3_transpose(U, Ut);

    double VUt[9];
    mat3_mul(V, Ut, VUt);

    double d = mat3_det(VUt);

    double R[9];
    if (d >= 0.0) {
        std::memcpy(R, VUt, 9 * sizeof(double));
    } else {
        // Reflection case: negate the column of V corresponding to
        // smallest singular value and recompute R = V' * U^T
        double V_corr[9];
        std::memcpy(V_corr, V, 9 * sizeof(double));
        for (int r = 0; r < 3; ++r)
            m3(V_corr, r, 2) = -m3(V_corr, r, 2);
        mat3_mul(V_corr, Ut, R);
    }

    // Step 5: t = c_tgt - R * c_src
    double Rc_src[3];
    mat3_vec(R, c_src, Rc_src);
    double t[3];
    for (int j = 0; j < 3; ++j)
        t[j] = c_tgt[j] - Rc_src[j];

    // Convert to quaternion
    double q[4];
    rotMatToQuat(R, q);

    result.transform.rotation[0] = q[0];
    result.transform.rotation[1] = q[1];
    result.transform.rotation[2] = q[2];
    result.transform.rotation[3] = q[3];
    result.transform.translation[0] = t[0];
    result.transform.translation[1] = t[1];
    result.transform.translation[2] = t[2];

    // Step 6: Compute RMSE
    double sse = 0.0;
    for (int i = 0; i < n_points; ++i) {
        double Rp[3];
        mat3_vec(R, source + i * 3, Rp);
        for (int j = 0; j < 3; ++j) {
            double diff = target[i * 3 + j] - (Rp[j] + t[j]);
            sse += diff * diff;
        }
    }
    result.rmse = std::sqrt(sse / static_cast<double>(n_points));
    result.num_inliers = n_points;
    result.valid = true;

    return result;
}

RigidTransformResult solveRigidTransform(const std::vector<double>& source,
                                         const std::vector<double>& target) {
    int n = static_cast<int>(std::min(source.size(), target.size()) / 3);
    return solveRigidTransform(source.data(), target.data(), n);
}

RigidTransformResult solveRigidTransformRANSAC(
    const double* source, const double* target, int n_points,
    double threshold, int max_iter, double inlier_ratio) {

    RigidTransformResult best;
    if (!source || !target || n_points < 3) return best;

    // For small point sets, just use direct SVD
    if (n_points <= 6) return solveRigidTransform(source, target, n_points);

    std::mt19937 rng(42);  // Deterministic seed for reproducibility
    double threshold_sq = threshold * threshold;
    int best_inliers = 0;

    for (int iter = 0; iter < max_iter; ++iter) {
        // Sample 3 random points (minimum for rigid transform)
        int idx[3];
        idx[0] = static_cast<int>(rng() % static_cast<unsigned>(n_points));
        do { idx[1] = static_cast<int>(rng() % static_cast<unsigned>(n_points)); }
        while (idx[1] == idx[0]);
        do { idx[2] = static_cast<int>(rng() % static_cast<unsigned>(n_points)); }
        while (idx[2] == idx[0] || idx[2] == idx[1]);

        // Solve with minimal set
        double src_sub[9], tgt_sub[9];
        for (int k = 0; k < 3; ++k) {
            for (int j = 0; j < 3; ++j) {
                src_sub[k * 3 + j] = source[idx[k] * 3 + j];
                tgt_sub[k * 3 + j] = target[idx[k] * 3 + j];
            }
        }

        auto candidate = solveRigidTransform(src_sub, tgt_sub, 3);
        if (!candidate.valid) continue;

        // Count inliers: apply transform and check residuals
        // Convert quaternion back to rotation matrix for fast application
        double q[4] = { candidate.transform.rotation[0], candidate.transform.rotation[1],
                         candidate.transform.rotation[2], candidate.transform.rotation[3] };
        double t[3] = { candidate.transform.translation[0],
                         candidate.transform.translation[1],
                         candidate.transform.translation[2] };

        // Quaternion to rotation matrix
        double w = q[0], x = q[1], y = q[2], z = q[3];
        double R[9] = {
            1.0 - 2.0*(y*y + z*z), 2.0*(x*y - w*z),       2.0*(x*z + w*y),
            2.0*(x*y + w*z),       1.0 - 2.0*(x*x + z*z), 2.0*(y*z - w*x),
            2.0*(x*z - w*y),       2.0*(y*z + w*x),       1.0 - 2.0*(x*x + y*y)
        };

        int inliers = 0;
        for (int i = 0; i < n_points; ++i) {
            double Rp[3];
            mat3_vec(R, source + i * 3, Rp);
            double dist_sq = 0.0;
            for (int j = 0; j < 3; ++j) {
                double d = target[i * 3 + j] - (Rp[j] + t[j]);
                dist_sq += d * d;
            }
            if (dist_sq < threshold_sq) ++inliers;
        }

        if (inliers > best_inliers) {
            best_inliers = inliers;

            // Refit using all inliers
            std::vector<double> inlier_src, inlier_tgt;
            inlier_src.reserve(static_cast<size_t>(inliers) * 3);
            inlier_tgt.reserve(static_cast<size_t>(inliers) * 3);

            for (int i = 0; i < n_points; ++i) {
                double Rp[3];
                mat3_vec(R, source + i * 3, Rp);
                double dist_sq = 0.0;
                for (int j = 0; j < 3; ++j) {
                    double d = target[i * 3 + j] - (Rp[j] + t[j]);
                    dist_sq += d * d;
                }
                if (dist_sq < threshold_sq) {
                    for (int j = 0; j < 3; ++j) {
                        inlier_src.push_back(source[i * 3 + j]);
                        inlier_tgt.push_back(target[i * 3 + j]);
                    }
                }
            }

            best = solveRigidTransform(inlier_src.data(), inlier_tgt.data(), inliers);

            // Early termination if we have enough inliers
            if (static_cast<double>(inliers) / static_cast<double>(n_points) > 0.95)
                break;
        }
    }

    // Check minimum inlier ratio
    if (static_cast<double>(best.num_inliers) / static_cast<double>(n_points)
        < inlier_ratio) {
        best.valid = false;
    }

    return best;
}

}  // namespace thunderbird::calib
