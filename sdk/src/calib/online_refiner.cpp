// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Online Extrinsic Refinement Implementation
// ─────────────────────────────────────────────────────────────────────────────
#include "online_refiner.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace thunderbird::calib {
namespace {

// ─── 3×3 symmetric eigendecomposition (Jacobi) ────────────────────────────
//
// Eigenvalues returned in ascending order; eigenvectors as columns.

struct Eigen3Result {
    double values[3];      // eigenvalues, ascending
    double vectors[9];     // column-major eigenvectors
};

inline double& col(double* M, int r, int c) { return M[c * 3 + r]; }  // column-major

Eigen3Result symEigen3(const double S[6]) {
    // S is stored as [s00, s01, s02, s11, s12, s22] (upper triangle)
    // Expand to full symmetric 3×3
    double A[9];  // column-major
    col(A, 0, 0) = S[0]; col(A, 0, 1) = S[1]; col(A, 0, 2) = S[2];
    col(A, 1, 0) = S[1]; col(A, 1, 1) = S[3]; col(A, 1, 2) = S[4];
    col(A, 2, 0) = S[2]; col(A, 2, 1) = S[4]; col(A, 2, 2) = S[5];

    // V = I (eigenvector accumulator)
    double V[9];
    std::memset(V, 0, sizeof(V));
    col(V, 0, 0) = col(V, 1, 1) = col(V, 2, 2) = 1.0;

    // Jacobi rotations
    for (int iter = 0; iter < 50; ++iter) {
        // Find largest off-diagonal
        double max_off = 0.0;
        int p = 0, q = 1;
        for (int r = 0; r < 3; ++r)
            for (int c = r + 1; c < 3; ++c)
                if (std::fabs(col(A, r, c)) > max_off) {
                    max_off = std::fabs(col(A, r, c));
                    p = r; q = c;
                }

        if (max_off < 1e-15) break;

        double app = col(A, p, p), aqq = col(A, q, q), apq = col(A, p, q);
        double theta;
        if (std::fabs(app - aqq) < 1e-15)
            theta = M_PI / 4.0;
        else
            theta = 0.5 * std::atan2(2.0 * apq, app - aqq);

        double c = std::cos(theta), s = std::sin(theta);

        // Apply Givens rotation to A (column-major)
        // A <- G^T A G
        double new_A[9];
        std::memcpy(new_A, A, sizeof(A));

        for (int i = 0; i < 3; ++i) {
            col(new_A, i, p) =  c * col(A, i, p) + s * col(A, i, q);
            col(new_A, i, q) = -s * col(A, i, p) + c * col(A, i, q);
        }
        std::memcpy(A, new_A, sizeof(A));
        for (int i = 0; i < 3; ++i) {
            col(A, p, i) =  c * col(new_A, p, i) + s * col(new_A, q, i);
            col(A, q, i) = -s * col(new_A, p, i) + c * col(new_A, q, i);
        }

        // Accumulate V
        double new_V[9];
        std::memcpy(new_V, V, sizeof(V));
        for (int i = 0; i < 3; ++i) {
            col(new_V, i, p) =  c * col(V, i, p) + s * col(V, i, q);
            col(new_V, i, q) = -s * col(V, i, p) + c * col(V, i, q);
        }
        std::memcpy(V, new_V, sizeof(V));
    }

    // Sort ascending by eigenvalue, rearrange eigenvectors
    Eigen3Result result;
    int idx[3] = {0, 1, 2};
    double eig[3] = { col(A, 0, 0), col(A, 1, 1), col(A, 2, 2) };
    if (eig[idx[0]] > eig[idx[1]]) std::swap(idx[0], idx[1]);
    if (eig[idx[0]] > eig[idx[2]]) std::swap(idx[0], idx[2]);
    if (eig[idx[1]] > eig[idx[2]]) std::swap(idx[1], idx[2]);

    for (int c = 0; c < 3; ++c) {
        result.values[c] = eig[idx[c]];
        for (int r = 0; r < 3; ++r)
            col(result.vectors, r, c) = col(V, r, idx[c]);
    }
    return result;
}

// ─── Quaternion helpers ────────────────────────────────────────────────────

void quatNormalize(double q[4]) {
    double n = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n > 0) { q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n; }
}

// Quaternion SLERP: interpolate from q0 to q1 by factor t
void quatSlerp(const double q0[4], const double q1[4], double t, double out[4]) {
    double dot = q0[0]*q1[0] + q0[1]*q1[1] + q0[2]*q1[2] + q0[3]*q1[3];

    double q1_adj[4] = { q1[0], q1[1], q1[2], q1[3] };
    if (dot < 0.0) {
        dot = -dot;
        q1_adj[0] = -q1_adj[0]; q1_adj[1] = -q1_adj[1];
        q1_adj[2] = -q1_adj[2]; q1_adj[3] = -q1_adj[3];
    }

    if (dot > 0.9995) {
        // Linear interpolation for very close quaternions
        for (int i = 0; i < 4; ++i)
            out[i] = q0[i] + t * (q1_adj[i] - q0[i]);
        quatNormalize(out);
        return;
    }

    double theta = std::acos(std::min(dot, 1.0));
    double sin_theta = std::sin(theta);
    double w0 = std::sin((1.0 - t) * theta) / sin_theta;
    double w1 = std::sin(t * theta) / sin_theta;
    for (int i = 0; i < 4; ++i)
        out[i] = w0 * q0[i] + w1 * q1_adj[i];
    quatNormalize(out);
}

// Rotation matrix from axis-angle (axis must be unit vector)
void axisAngleToQuat(const double axis[3], double angle, double q[4]) {
    double ha = angle * 0.5;
    double sha = std::sin(ha);
    q[0] = std::cos(ha);
    q[1] = axis[0] * sha;
    q[2] = axis[1] * sha;
    q[3] = axis[2] * sha;
    quatNormalize(q);
}

}  // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────

OnlineRefiner::OnlineRefiner(const OnlineRefinerConfig& config)
    : config_(config) {}

OnlineRefiner::~OnlineRefiner() = default;

bool OnlineRefiner::processFrame(const RefinerPoint* points, int n_points) {
    if (!points || n_points < config_.min_ground_points) return false;

    // Step 1: Extract ground candidate points (below height threshold)
    std::vector<const RefinerPoint*> ground;
    ground.reserve(static_cast<size_t>(n_points / 4));
    for (int i = 0; i < n_points; ++i) {
        if (points[i].z < config_.ground_max_height)
            ground.push_back(&points[i]);
    }

    if (static_cast<int>(ground.size()) < config_.min_ground_points)
        return false;

    // Step 2: Compute centroid
    double cx = 0, cy = 0, cz = 0;
    for (const auto* p : ground) { cx += p->x; cy += p->y; cz += p->z; }
    double inv_n = 1.0 / static_cast<double>(ground.size());
    cx *= inv_n; cy *= inv_n; cz *= inv_n;

    // Step 3: Compute covariance (upper triangle: s00,s01,s02,s11,s12,s22)
    double S[6] = {};
    for (const auto* p : ground) {
        double dx = p->x - cx, dy = p->y - cy, dz = p->z - cz;
        S[0] += dx * dx; S[1] += dx * dy; S[2] += dx * dz;
        S[3] += dy * dy; S[4] += dy * dz;
        S[5] += dz * dz;
    }

    // Step 4: Eigendecomposition → ground normal = smallest eigenvector
    auto eig = symEigen3(S);
    double normal[3] = { col(eig.vectors, 0, 0),
                         col(eig.vectors, 1, 0),
                         col(eig.vectors, 2, 0) };

    // Ensure normal points upward (positive Z in LiDAR frame)
    if (normal[2] < 0) { normal[0] = -normal[0]; normal[1] = -normal[1]; normal[2] = -normal[2]; }

    // Check normal quality
    if (normal[2] < config_.min_normal_z) return false;

    // Step 5: Ground intercept = -(normal · centroid) / normal_z
    double intercept = -(normal[0]*cx + normal[1]*cy + normal[2]*cz);
    double height = intercept / normal[2];
    if (height < config_.min_height) return false;

    // Step 6: Compute rotation that aligns ground normal to [0,0,1]
    double up[3] = {0, 0, 1};
    // Cross product: axis = normal × up
    double axis[3] = {
        normal[1] * up[2] - normal[2] * up[1],
        normal[2] * up[0] - normal[0] * up[2],
        normal[0] * up[1] - normal[1] * up[0]
    };
    double axis_len = std::sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);

    double q_frame[4] = {1, 0, 0, 0};
    if (axis_len > 1e-8) {
        axis[0] /= axis_len; axis[1] /= axis_len; axis[2] /= axis_len;
        double angle = std::acos(std::min(1.0,
            normal[0]*up[0] + normal[1]*up[1] + normal[2]*up[2]));
        axisAngleToQuat(axis, angle, q_frame);
    }

    // Step 7: Confidence-weighted accumulation via SLERP
    double confidence = height * height - config_.min_height;
    if (confidence < 0) confidence = 0;

    double q_scale = (correction_.confidence == 0.0)
        ? 1.0
        : confidence / (correction_.confidence + confidence);

    double q_accumulated[4];
    if (correction_.frame_count == 0) {
        std::memcpy(q_accumulated, q_frame, sizeof(q_frame));
    } else {
        quatSlerp(correction_.rotation, q_frame, q_scale, q_accumulated);
    }

    // Update height with same weighting
    double new_height = q_scale * height + (1.0 - q_scale) * correction_.height;

    // Step 8: Store updated correction
    std::memcpy(correction_.rotation, q_accumulated, sizeof(q_accumulated));
    correction_.height = new_height;
    correction_.confidence += confidence;
    correction_.frame_count++;

    // Check safety thresholds
    // Compute angle from identity quaternion
    double angle_from_id = 2.0 * std::acos(std::min(1.0, std::fabs(correction_.rotation[0])));
    double angle_deg = angle_from_id * 180.0 / M_PI;
    const double height_delta = correction_.height - config_.min_height;
    correction_.warning = (angle_deg > config_.max_correction_deg) ||
                          (std::fabs(height_delta) > config_.max_correction_height);

    return true;
}

ExtrinsicCorrection OnlineRefiner::correction() const {
    return correction_;
}

void OnlineRefiner::reset() {
    correction_ = ExtrinsicCorrection{};
}

}  // namespace thunderbird::calib
