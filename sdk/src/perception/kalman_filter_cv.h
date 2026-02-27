// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Constant-Velocity Kalman Filter (7-state)
// ─────────────────────────────────────────────────────────────────────────────
//
// Per-track Extended Kalman Filter with state vector:
//
//   x = [px, py, pz, yaw, vx, vy, vz]ᵀ   (7 × 1)
//
// Measurement vector (from detector):
//
//   z = [px, py, pz, yaw]ᵀ               (4 × 1)
//
// Motion model (constant velocity):
//   px' = px + vx·dt
//   py' = py + vy·dt
//   pz' = pz + vz·dt
//   yaw'= yaw               (no rotation dynamics in CV mode)
//   vx' = vx
//   vy' = vy
//   vz' = vz
//
// All matrices are stored as flat arrays in row-major order.
// No Eigen dependency — pure standard C++.
//
// This is an internal (non-public) header used only by multi_object_tracker.cpp.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <cmath>
#include <cstring>
#include <array>

namespace thunderbird::perception::detail {

static constexpr int KF_STATE_DIM = 7;   // px, py, pz, yaw, vx, vy, vz
static constexpr int KF_MEAS_DIM  = 4;   // px, py, pz, yaw

// ═════════════════════════════════════════════════════════════════════════════
//  Tiny linear algebra helpers (7×7 max)
// ═════════════════════════════════════════════════════════════════════════════

namespace linalg {

/// C = A × B  where A is (rA×cA) and B is (cA×cB).
inline void mat_mul(const double* A, const double* B,
                    double* C, int rA, int cA, int cB) {
    for (int i = 0; i < rA; ++i) {
        for (int j = 0; j < cB; ++j) {
            double sum = 0;
            for (int k = 0; k < cA; ++k) {
                sum += A[i * cA + k] * B[k * cB + j];
            }
            C[i * cB + j] = sum;
        }
    }
}

/// C = Aᵀ  where A is (rows×cols).
inline void mat_transpose(const double* A, double* AT, int rows, int cols) {
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; ++j)
            AT[j * rows + i] = A[i * cols + j];
}

/// C = A + B  (n×n).
inline void mat_add(const double* A, const double* B, double* C, int n) {
    for (int i = 0; i < n * n; ++i) C[i] = A[i] + B[i];
}

/// C = A − B  (n×n or n×m stored flat).
inline void mat_sub(const double* A, const double* B, double* C, int len) {
    for (int i = 0; i < len; ++i) C[i] = A[i] - B[i];
}

/// Set A to n×n identity.
inline void mat_eye(double* A, int n) {
    std::memset(A, 0, sizeof(double) * static_cast<size_t>(n * n));
    for (int i = 0; i < n; ++i) A[i * n + i] = 1.0;
}

/// Invert a small matrix (up to 7×7) using Gauss-Jordan elimination.
/// Returns false if singular.
inline bool mat_inv(const double* A, double* Ainv, int n) {
    // Augmented matrix [A | I]
    double aug[7 * 14]{};
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j)
            aug[i * (2 * n) + j] = A[i * n + j];
        aug[i * (2 * n) + n + i] = 1.0;
    }

    for (int col = 0; col < n; ++col) {
        // Partial pivot.
        int max_row = col;
        double max_val = std::fabs(aug[col * (2 * n) + col]);
        for (int row = col + 1; row < n; ++row) {
            double v = std::fabs(aug[row * (2 * n) + col]);
            if (v > max_val) { max_val = v; max_row = row; }
        }
        if (max_val < 1e-14) return false;  // singular

        // Swap rows.
        if (max_row != col) {
            for (int j = 0; j < 2 * n; ++j)
                std::swap(aug[col * (2 * n) + j], aug[max_row * (2 * n) + j]);
        }

        // Scale pivot row.
        const double pivot = aug[col * (2 * n) + col];
        for (int j = 0; j < 2 * n; ++j)
            aug[col * (2 * n) + j] /= pivot;

        // Eliminate column.
        for (int row = 0; row < n; ++row) {
            if (row == col) continue;
            const double factor = aug[row * (2 * n) + col];
            for (int j = 0; j < 2 * n; ++j)
                aug[row * (2 * n) + j] -= factor * aug[col * (2 * n) + j];
        }
    }

    // Extract inverse.
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            Ainv[i * n + j] = aug[i * (2 * n) + n + j];

    return true;
}

} // namespace linalg

// ═════════════════════════════════════════════════════════════════════════════
//  KalmanFilterCV — 7-state constant-velocity filter
// ═════════════════════════════════════════════════════════════════════════════

class KalmanFilterCV {
public:
    /// Initialize filter with first measurement.
    void init(double px, double py, double pz, double yaw,
              double sigma_pos, double sigma_vel, double sigma_yaw) {
        x_[0] = px;
        x_[1] = py;
        x_[2] = pz;
        x_[3] = yaw;
        x_[4] = 0.0;  // vx
        x_[5] = 0.0;  // vy
        x_[6] = 0.0;  // vz

        linalg::mat_eye(P_, N);

        // Initial uncertainty: high for velocity, moderate for position.
        P_[0 * N + 0] = sigma_pos * sigma_pos;   // px
        P_[1 * N + 1] = sigma_pos * sigma_pos;   // py
        P_[2 * N + 2] = sigma_pos * sigma_pos;   // pz
        P_[3 * N + 3] = sigma_yaw * sigma_yaw;   // yaw
        P_[4 * N + 4] = sigma_vel * sigma_vel * 4.0;  // vx (high uncertainty)
        P_[5 * N + 5] = sigma_vel * sigma_vel * 4.0;  // vy
        P_[6 * N + 6] = sigma_vel * sigma_vel * 4.0;  // vz
    }

    /// Predict forward by dt seconds.
    void predict(double dt,
                 double q_pos, double q_vel, double q_yaw) {
        // State transition: x' = F·x
        // F = I + dt * [0 0 0 0 1 0 0;
        //               0 0 0 0 0 1 0;
        //               0 0 0 0 0 0 1;
        //               0 ...       0;
        //               0 ...       0; ...]
        x_[0] += x_[4] * dt;  // px += vx * dt
        x_[1] += x_[5] * dt;  // py += vy * dt
        x_[2] += x_[6] * dt;  // pz += vz * dt
        // yaw and velocities unchanged in CV model.

        // Build F matrix.
        double F[N * N];
        linalg::mat_eye(F, N);
        F[0 * N + 4] = dt;  // dx/dvx
        F[1 * N + 5] = dt;  // dy/dvy
        F[2 * N + 6] = dt;  // dz/dvz

        // Process noise Q (diagonal, dt-scaled).
        double Q[N * N];
        std::memset(Q, 0, sizeof(Q));
        const double dt2 = dt * dt;
        // Position: both direct noise + coupled velocity noise.
        Q[0 * N + 0] = q_pos * q_pos * dt2 + q_vel * q_vel * dt2 * dt2 / 4.0;
        Q[1 * N + 1] = Q[0 * N + 0];
        Q[2 * N + 2] = Q[0 * N + 0];
        Q[3 * N + 3] = q_yaw * q_yaw * dt2;
        // Velocity noise.
        Q[4 * N + 4] = q_vel * q_vel * dt2;
        Q[5 * N + 5] = Q[4 * N + 4];
        Q[6 * N + 6] = Q[4 * N + 4];
        // Cross-terms (position-velocity coupling).
        const double cross = q_vel * q_vel * dt2 * dt / 2.0;
        Q[0 * N + 4] = cross; Q[4 * N + 0] = cross;
        Q[1 * N + 5] = cross; Q[5 * N + 1] = cross;
        Q[2 * N + 6] = cross; Q[6 * N + 2] = cross;

        // P' = F·P·Fᵀ + Q
        double FP[N * N], FT[N * N], FPFt[N * N];
        linalg::mat_mul(F, P_, FP, N, N, N);
        linalg::mat_transpose(F, FT, N, N);
        linalg::mat_mul(FP, FT, FPFt, N, N, N);
        linalg::mat_add(FPFt, Q, P_, N);
    }

    /// Update with measurement z = [px, py, pz, yaw].
    void update(const double z[4], double r_pos, double r_yaw) {
        // Measurement matrix H (M × N), selecting position and yaw from state:
        //   | 1 0 0 0 0 0 0 |
        //   | 0 1 0 0 0 0 0 |
        //   | 0 0 1 0 0 0 0 |
        //   | 0 0 0 1 0 0 0 |
        double H[M * N];
        std::memset(H, 0, sizeof(H));
        H[0 * N + 0] = 1.0;
        H[1 * N + 1] = 1.0;
        H[2 * N + 2] = 1.0;
        H[3 * N + 3] = 1.0;

        // Measurement noise R.
        double R[M * M];
        std::memset(R, 0, sizeof(R));
        R[0 * M + 0] = r_pos * r_pos;
        R[1 * M + 1] = r_pos * r_pos;
        R[2 * M + 2] = r_pos * r_pos;
        R[3 * M + 3] = r_yaw * r_yaw;

        // Innovation: y = z - H·x
        double Hx[M];
        double y[M];
        for (int i = 0; i < M; ++i) {
            Hx[i] = 0;
            for (int j = 0; j < N; ++j)
                Hx[i] += H[i * N + j] * x_[j];
            y[i] = z[i] - Hx[i];
        }

        // Normalise yaw innovation to [-π, π].
        while (y[3] >  M_PI) y[3] -= 2.0 * M_PI;
        while (y[3] < -M_PI) y[3] += 2.0 * M_PI;

        // Innovation covariance: S = H·P·Hᵀ + R  (M × M)
        double HT[N * M], HP[M * N], S[M * M];
        linalg::mat_transpose(H, HT, M, N);
        linalg::mat_mul(H, P_, HP, M, N, N);
        linalg::mat_mul(HP, HT, S, M, M, M);
        linalg::mat_add(S, R, S, M);

        // Kalman gain: K = P·Hᵀ·S⁻¹  (N × M)
        double Sinv[M * M];
        if (!linalg::mat_inv(S, Sinv, M)) {
            // Singular — skip update, keep prediction.
            return;
        }

        double PHt[N * M], K[N * M];
        linalg::mat_mul(P_, HT, PHt, N, N, M);
        linalg::mat_mul(PHt, Sinv, K, N, M, M);

        // State update: x = x + K·y
        for (int i = 0; i < N; ++i) {
            double ky = 0;
            for (int j = 0; j < M; ++j)
                ky += K[i * M + j] * y[j];
            x_[i] += ky;
        }

        // Normalise yaw state.
        while (x_[3] >  M_PI) x_[3] -= 2.0 * M_PI;
        while (x_[3] < -M_PI) x_[3] += 2.0 * M_PI;

        // Covariance update: P = (I − K·H)·P
        // Use Joseph form for numerical stability:
        //   P = (I - K·H)·P·(I - K·H)ᵀ + K·R·Kᵀ
        double KH[N * N];
        linalg::mat_mul(K, H, KH, N, M, N);

        double IKH[N * N];
        linalg::mat_eye(IKH, N);
        for (int i = 0; i < N * N; ++i) IKH[i] -= KH[i];

        double IKHt[N * N];
        linalg::mat_transpose(IKH, IKHt, N, N);

        double IKH_P[N * N], P_new[N * N];
        linalg::mat_mul(IKH, P_, IKH_P, N, N, N);
        linalg::mat_mul(IKH_P, IKHt, P_new, N, N, N);

        double KR[N * M], Kt[N * N], KRKt[N * N];  // Kt padded to N×N to silence GCC
        linalg::mat_mul(K, R, KR, N, M, M);
        linalg::mat_transpose(K, Kt, N, M);
        linalg::mat_mul(KR, Kt, KRKt, N, M, N);

        linalg::mat_add(P_new, KRKt, P_, N);
    }

    /// Compute Mahalanobis distance between current state and measurement z.
    [[nodiscard]] double mahalanobis(const std::array<double, M>& z,
                                     double r_pos, double r_yaw) const {
        // Innovation: y = z − H·x  (H extracts first 4 state elements).
        double y[M];
        y[0] = z[0] - x_[0];
        y[1] = z[1] - x_[1];
        y[2] = z[2] - x_[2];
        y[3] = z[3] - x_[3];
        while (y[3] >  M_PI) y[3] -= 2.0 * M_PI;
        while (y[3] < -M_PI) y[3] += 2.0 * M_PI;

        // S = H·P·Hᵀ + R — for this simple H, S = P[0:4, 0:4] + R.
        double S[M * M];
        for (int i = 0; i < M; ++i)
            for (int j = 0; j < M; ++j)
                S[i * M + j] = P_[i * N + j];

        double R[M * M]{};
        R[0 * M + 0] = r_pos * r_pos;
        R[1 * M + 1] = r_pos * r_pos;
        R[2 * M + 2] = r_pos * r_pos;
        R[3 * M + 3] = r_yaw * r_yaw;

        linalg::mat_add(S, R, S, M);

        double Sinv[M * M];
        if (!linalg::mat_inv(S, Sinv, M)) {
            return 1e9;  // worst case if singular
        }

        // d² = yᵀ·S⁻¹·y
        double d2 = 0;
        for (int i = 0; i < M; ++i)
            for (int j = 0; j < M; ++j)
                d2 += y[i] * Sinv[i * M + j] * y[j];

        return std::sqrt(std::fabs(d2));
    }

    // ── Accessors ───────────────────────────────────────────────────────

    [[nodiscard]] double px()  const noexcept { return x_[0]; }
    [[nodiscard]] double py()  const noexcept { return x_[1]; }
    [[nodiscard]] double pz()  const noexcept { return x_[2]; }
    [[nodiscard]] double yaw() const noexcept { return x_[3]; }
    [[nodiscard]] double vx()  const noexcept { return x_[4]; }
    [[nodiscard]] double vy()  const noexcept { return x_[5]; }
    [[nodiscard]] double vz()  const noexcept { return x_[6]; }

    [[nodiscard]] const double* state()      const noexcept { return x_; }
    [[nodiscard]] const double* covariance() const noexcept { return P_; }

    /// Copy the 7×7 covariance into user-provided buffer.
    void copyCovariance(double out[49]) const noexcept {
        std::memcpy(out, P_, sizeof(P_));
    }

private:
    static constexpr int N = KF_STATE_DIM;   // 7
    static constexpr int M = KF_MEAS_DIM;    // 4

    double x_[N]{};         // state vector
    double P_[N * N]{};     // covariance matrix (row-major)
};

} // namespace thunderbird::perception::detail
