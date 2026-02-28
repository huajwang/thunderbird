// ─────────────────────────────────────────────────────────────────────────────
// Test — Kalman Filter (predict/update) & Hungarian assignment
// ─────────────────────────────────────────────────────────────────────────────
//
// Directly tests internal components that the MultiObjectTracker depends on:
//
//   Kalman filter (KalmanFilterCV):
//     • init sets state to given position, zero velocity
//     • predict propagates position by velocity × dt
//     • update corrects state towards measurement
//     • predict + update converges on a constant-velocity target
//     • Mahalanobis distance is small for nearby measurement, large for far
//     • Covariance grows during predict-only, shrinks on update
//
//   Hungarian algorithm:
//     • 0×0 matrix → no assignments
//     • 1×1 matrix → single match (if within gate)
//     • 1×1 matrix → no match (if beyond gate)
//     • Square 3×3 matrix — optimal assignment
//     • Rectangular (more dets than tracks, and vice versa)
//     • Gate filters out high-cost pairs
//
// ─────────────────────────────────────────────────────────────────────────────

// Internal headers — not part of public SDK but testable
#include "kalman_filter_cv.h"
#include "hungarian.h"

#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace thunderbird::perception::detail;

// ─────────────────────────────────────────────────────────────────────────────
//  Kalman filter tests
// ─────────────────────────────────────────────────────────────────────────────

static void test_kf_init() {
    KalmanFilterCV kf;
    kf.init(1.0, 2.0, 3.0, 0.5, /*sigma_pos=*/1.0, /*sigma_vel=*/2.0, /*sigma_yaw=*/0.1);

    assert(std::abs(kf.px() - 1.0) < 1e-9);
    assert(std::abs(kf.py() - 2.0) < 1e-9);
    assert(std::abs(kf.pz() - 3.0) < 1e-9);
    assert(std::abs(kf.yaw() - 0.5) < 1e-9);
    assert(std::abs(kf.vx()) < 1e-9);
    assert(std::abs(kf.vy()) < 1e-9);
    assert(std::abs(kf.vz()) < 1e-9);

    std::puts("  [PASS] kf_init");
}

static void test_kf_predict_moves_position() {
    KalmanFilterCV kf;
    kf.init(0, 0, 0, 0, 1.0, 1.0, 0.1);

    // A single update from a diagonal-P state doesn't build velocity
    // (no pos-vel cross-covariance yet).  We need a predict-update cycle
    // so that the predict step creates cross-terms, and the second update
    // infers velocity from the position shift.
    std::array<double, KF_MEAS_DIM> z1 = {{0.0, 0.0, 0.0, 0.0}};
    kf.update(z1, 0.5, 0.1);
    kf.predict(0.1, 0.5, 1.0, 0.1);  // builds pos-vel cross-covariance

    std::array<double, KF_MEAS_DIM> z2 = {{5.0, 0.0, 0.0, 0.0}};
    kf.update(z2, 0.5, 0.1);         // large innovation → positive vx

    assert(kf.vx() > 0.0);           // velocity should now be positive

    const double px_before = kf.px();
    kf.predict(1.0, 0.5, 1.0, 0.1);
    const double px_after = kf.px();

    // With positive velocity, predict should advance position
    assert(px_after > px_before);

    std::puts("  [PASS] kf_predict_moves_position");
}

static void test_kf_update_corrects_state() {
    KalmanFilterCV kf;
    kf.init(0, 0, 0, 0, 1.0, 1.0, 0.1);

    // Feed repeated measurements at (10, 5, 2, 0).  Even if a single
    // update is skipped (e.g. singular S on a platform), repeated
    // predict-update cycles will move the state towards the measurement.
    for (int i = 0; i < 5; ++i) {
        if (i > 0) kf.predict(0.1, 0.5, 1.0, 0.1);
        std::array<double, KF_MEAS_DIM> z = {{10.0, 5.0, 2.0, 0.0}};
        kf.update(z, 0.5, 0.1);
    }

    // State should have converged towards measurement
    assert(kf.px() > 1.0);
    assert(kf.py() > 0.5);
    assert(kf.pz() > 0.2);

    std::puts("  [PASS] kf_update_corrects_state");
}

static void test_kf_convergence_constant_velocity() {
    KalmanFilterCV kf;
    kf.init(0, 0, 0, 0, 1.0, 2.0, 0.1);

    const double true_vx = 3.0;  // m/s
    const double dt = 0.1;       // 10 Hz

    // Simulate 20 frames of a target moving at constant velocity
    for (int i = 1; i <= 20; ++i) {
        kf.predict(dt, 0.5, 1.0, 0.1);

        double true_x = true_vx * i * dt;
        std::array<double, KF_MEAS_DIM> z = {{true_x, 0.0, 0.0, 0.0}};
        kf.update(z, 0.3, 0.1);
    }

    // After 20 frames, velocity estimate should be close to true
    assert(std::abs(kf.vx() - true_vx) < true_vx * 0.5);
    // Position should be close to true position at frame 20
    assert(std::abs(kf.px() - true_vx * 20 * dt) < 2.0);

    std::puts("  [PASS] kf_convergence_constant_velocity");
}

static void test_kf_mahalanobis_near_vs_far() {
    KalmanFilterCV kf;
    kf.init(0, 0, 0, 0, 1.0, 1.0, 0.1);

    // Measurement very close to state
    std::array<double, KF_MEAS_DIM> z_near = {{0.1, 0.0, 0.0, 0.0}};
    double d_near = kf.mahalanobis(z_near, 0.5, 0.1);

    // Measurement far from state
    std::array<double, KF_MEAS_DIM> z_far = {{50.0, 50.0, 50.0, 3.0}};
    double d_far = kf.mahalanobis(z_far, 0.5, 0.1);

    assert(d_near < d_far);
    assert(d_near < 5.0);   // should be small
    assert(d_far > 5.0);    // should be large

    std::puts("  [PASS] kf_mahalanobis_near_vs_far");
}

static void test_kf_covariance_growth_and_shrink() {
    KalmanFilterCV kf;
    kf.init(0, 0, 0, 0, 1.0, 1.0, 0.1);

    // Get initial position covariance (P[0,0])
    const double* P0 = kf.covariance();
    const double cov_init = P0[0];  // P[0][0]

    // Predict without update — covariance should grow
    kf.predict(1.0, 0.5, 1.0, 0.1);
    const double* P1 = kf.covariance();
    const double cov_after_predict = P1[0];
    assert(cov_after_predict > cov_init);

    // Update — covariance should shrink
    std::array<double, KF_MEAS_DIM> z = {{0.0, 0.0, 0.0, 0.0}};
    kf.update(z, 0.3, 0.1);
    const double* P2 = kf.covariance();
    const double cov_after_update = P2[0];
    assert(cov_after_update < cov_after_predict);

    std::puts("  [PASS] kf_covariance_growth_and_shrink");
}

static void test_kf_yaw_wrapping() {
    KalmanFilterCV kf;
    kf.init(0, 0, 0, 3.0, 1.0, 1.0, 0.1);

    // Measure at yaw = -3.0 (which is ~3.28 rad away unwrapped, but
    // only ~0.28 rad away via wrapping through ±π)
    std::array<double, KF_MEAS_DIM> z = {{0.0, 0.0, 0.0, -3.0}};
    kf.update(z, 0.5, 0.1);

    // Yaw should remain in [-π, π]
    assert(kf.yaw() >= -M_PI && kf.yaw() <= M_PI);

    std::puts("  [PASS] kf_yaw_wrapping");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Hungarian algorithm tests
// ─────────────────────────────────────────────────────────────────────────────

static void test_hungarian_empty() {
    std::vector<double> cost;
    std::vector<Assignment> matches;
    std::vector<size_t> um_rows, um_cols;

    hungarian_solve(cost, 0, 0, 5.0, matches, um_rows, um_cols);
    assert(matches.empty());
    assert(um_rows.empty());
    assert(um_cols.empty());

    std::puts("  [PASS] hungarian_empty");
}

static void test_hungarian_1x1_match() {
    std::vector<double> cost = {2.0};
    std::vector<Assignment> matches;
    std::vector<size_t> um_rows, um_cols;

    hungarian_solve(cost, 1, 1, 5.0, matches, um_rows, um_cols);
    assert(matches.size() == 1);
    assert(matches[0].row == 0 && matches[0].col == 0);
    assert(um_rows.empty());
    assert(um_cols.empty());

    std::puts("  [PASS] hungarian_1x1_match");
}

static void test_hungarian_1x1_gated_out() {
    std::vector<double> cost = {10.0};  // cost > gate
    std::vector<Assignment> matches;
    std::vector<size_t> um_rows, um_cols;

    hungarian_solve(cost, 1, 1, 5.0, matches, um_rows, um_cols);
    assert(matches.empty());
    assert(um_rows.size() == 1 && um_rows[0] == 0);
    assert(um_cols.size() == 1 && um_cols[0] == 0);

    std::puts("  [PASS] hungarian_1x1_gated_out");
}

static void test_hungarian_3x3_optimal() {
    // Classic assignment problem:
    //   Cost:  det0→trk0=1  det0→trk1=9  det0→trk2=9
    //          det1→trk0=9  det1→trk1=1  det1→trk2=9
    //          det2→trk0=9  det2→trk1=9  det2→trk2=1
    // Optimal: (0,0), (1,1), (2,2) with total cost 3
    std::vector<double> cost = {
        1, 9, 9,
        9, 1, 9,
        9, 9, 1,
    };
    std::vector<Assignment> matches;
    std::vector<size_t> um_rows, um_cols;

    hungarian_solve(cost, 3, 3, 20.0, matches, um_rows, um_cols);
    assert(matches.size() == 3);
    // Each detection matched to same-index track
    for (const auto& m : matches) {
        assert(m.row == m.col);
    }
    assert(um_rows.empty());
    assert(um_cols.empty());

    std::puts("  [PASS] hungarian_3x3_optimal");
}

static void test_hungarian_more_dets_than_tracks() {
    // 3 dets, 2 tracks
    std::vector<double> cost = {
        1, 9,    // det0
        9, 1,    // det1
        9, 9,    // det2 (unmatched)
    };
    std::vector<Assignment> matches;
    std::vector<size_t> um_rows, um_cols;

    hungarian_solve(cost, 3, 2, 5.0, matches, um_rows, um_cols);
    assert(matches.size() == 2);
    assert(um_rows.size() == 1);  // det2 unmatched
    assert(um_cols.empty());

    std::puts("  [PASS] hungarian_more_dets_than_tracks");
}

static void test_hungarian_more_tracks_than_dets() {
    // 2 dets, 3 tracks
    std::vector<double> cost = {
        1, 9, 9,  // det0
        9, 1, 9,  // det1
    };
    std::vector<Assignment> matches;
    std::vector<size_t> um_rows, um_cols;

    hungarian_solve(cost, 2, 3, 5.0, matches, um_rows, um_cols);
    assert(matches.size() == 2);
    assert(um_rows.empty());
    assert(um_cols.size() == 1);  // track 2 unmatched

    std::puts("  [PASS] hungarian_more_tracks_than_dets");
}

static void test_hungarian_no_rows() {
    std::vector<double> cost;
    std::vector<Assignment> matches;
    std::vector<size_t> um_rows, um_cols;

    hungarian_solve(cost, 0, 3, 5.0, matches, um_rows, um_cols);
    assert(matches.empty());
    assert(um_rows.empty());
    assert(um_cols.size() == 3);

    std::puts("  [PASS] hungarian_no_rows");
}

static void test_hungarian_no_cols() {
    std::vector<double> cost;
    std::vector<Assignment> matches;
    std::vector<size_t> um_rows, um_cols;

    hungarian_solve(cost, 3, 0, 5.0, matches, um_rows, um_cols);
    assert(matches.empty());
    assert(um_rows.size() == 3);
    assert(um_cols.empty());

    std::puts("  [PASS] hungarian_no_cols");
}

static void test_hungarian_partial_gate() {
    // 2×2 matrix — one pair is within gate, one is gated out
    std::vector<double> cost = {
        1, 9,   // det0→trk0 OK, det0→trk1 gated
        9, 1,   // det1→trk0 gated, det1→trk1 OK
    };
    std::vector<Assignment> matches;
    std::vector<size_t> um_rows, um_cols;

    // Gate = 5.0 → costs of 9 are gated out (but optimal picks 1s anyway)
    hungarian_solve(cost, 2, 2, 5.0, matches, um_rows, um_cols);
    assert(matches.size() == 2);
    assert(matches[0].row == 0 && matches[0].col == 0);
    assert(matches[1].row == 1 && matches[1].col == 1);

    std::puts("  [PASS] hungarian_partial_gate");
}

// ─────────────────────────────────────────────────────────────────────────────

int main() {
    std::puts("KalmanFilterCV tests:");

    test_kf_init();
    test_kf_predict_moves_position();
    test_kf_update_corrects_state();
    test_kf_convergence_constant_velocity();
    test_kf_mahalanobis_near_vs_far();
    test_kf_covariance_growth_and_shrink();
    test_kf_yaw_wrapping();

    std::puts("\nHungarian algorithm tests:");

    test_hungarian_empty();
    test_hungarian_1x1_match();
    test_hungarian_1x1_gated_out();
    test_hungarian_3x3_optimal();
    test_hungarian_more_dets_than_tracks();
    test_hungarian_more_tracks_than_dets();
    test_hungarian_no_rows();
    test_hungarian_no_cols();
    test_hungarian_partial_gate();

    std::puts("\nKalmanFilter & Hungarian: ALL TESTS PASSED");
    return 0;
}
