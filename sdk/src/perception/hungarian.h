// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Hungarian (Munkres) Algorithm
// ─────────────────────────────────────────────────────────────────────────────
//
// O(n³) implementation of the Hungarian algorithm for minimum-cost assignment
// of detections to tracks.  Operates on a rectangular cost matrix and returns
// the optimal (row, col) pairs that minimise total cost.
//
// This is an internal (non-public) header used only by multi_object_tracker.cpp.
//
// Reference:  Kuhn, H.W. (1955). "The Hungarian Method for the Assignment
//             Problem." Naval Research Logistics Quarterly, 2(1-2), 83–97.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <algorithm>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace thunderbird::perception::detail {

/// Result of a single assignment.
struct Assignment {
    size_t row;   // detection index
    size_t col;   // track index
};

/// Solve the minimum-cost assignment problem.
///
/// @param  cost_matrix  Row-major cost matrix [n_rows × n_cols].
///                      Rows = detections, columns = tracks.
///                      **Must** have at least max(n_rows, n_cols) ≥ 1.
/// @param  n_rows       Number of detections.
/// @param  n_cols       Number of tracks.
/// @param  gate         Maximum allowable cost.  Pairs whose original cost
///                      exceeds this are excluded from the returned matches.
/// @param[out] assignments        Matched (det, track) pairs.
/// @param[out] unmatched_rows     Detection indices with no match.
/// @param[out] unmatched_cols     Track indices with no match.
///
/// The algorithm pads the matrix to square with the gate value so that
/// unassignable elements are naturally rejected.
inline void hungarian_solve(
    const std::vector<double>& cost_matrix,
    size_t                     n_rows,
    size_t                     n_cols,
    double                     gate,
    std::vector<Assignment>&   assignments,
    std::vector<size_t>&       unmatched_rows,
    std::vector<size_t>&       unmatched_cols)
{
    assignments.clear();
    unmatched_rows.clear();
    unmatched_cols.clear();

    if (n_rows == 0 && n_cols == 0) return;

    // Handle degenerate cases.
    if (n_rows == 0) {
        for (size_t c = 0; c < n_cols; ++c) unmatched_cols.push_back(c);
        return;
    }
    if (n_cols == 0) {
        for (size_t r = 0; r < n_rows; ++r) unmatched_rows.push_back(r);
        return;
    }

    // ── Pad to square matrix ────────────────────────────────────────────
    const size_t n = std::max(n_rows, n_cols);
    const double PAD = gate + 1.0;  // cost that will never be chosen

    std::vector<double> mat(n * n, PAD);
    for (size_t r = 0; r < n_rows; ++r) {
        for (size_t c = 0; c < n_cols; ++c) {
            mat[r * n + c] = cost_matrix[r * n_cols + c];
        }
    }

    // ── Munkres (modified Jonker-Volgenant flavour) ─────────────────────
    // u[i], v[j]: dual variables (potentials)
    // p[j]:       row assigned to column j (0 = unassigned, 1-indexed)
    // way[j]:     augmenting path predecessor column for column j

    const size_t N = n + 1;  // 1-indexed
    std::vector<double> u(N, 0.0), v(N, 0.0);
    std::vector<size_t> p(N, 0), way(N, 0);

    for (size_t i = 1; i <= n; ++i) {
        // Start augmenting path from row i.
        p[0] = i;
        size_t j0 = 0;  // virtual column
        std::vector<double> minv(N, std::numeric_limits<double>::max());
        std::vector<bool> used(N, false);

        do {
            used[j0] = true;
            const size_t i0 = p[j0];
            double delta = std::numeric_limits<double>::max();
            size_t j1 = 0;

            for (size_t j = 1; j <= n; ++j) {
                if (used[j]) continue;
                const double cur = mat[(i0 - 1) * n + (j - 1)] - u[i0] - v[j];
                if (cur < minv[j]) {
                    minv[j] = cur;
                    way[j] = j0;
                }
                if (minv[j] < delta) {
                    delta = minv[j];
                    j1 = j;
                }
            }

            for (size_t j = 0; j <= n; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j]    -= delta;
                } else {
                    minv[j] -= delta;
                }
            }

            j0 = j1;
        } while (p[j0] != 0);

        // Update assignment along augmenting path.
        do {
            const size_t j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }

    // ── Extract results ─────────────────────────────────────────────────
    // p[j] = row assigned to column j  (1-indexed)
    std::vector<int> row_to_col(n, -1);
    for (size_t j = 1; j <= n; ++j) {
        if (p[j] != 0) {
            row_to_col[p[j] - 1] = static_cast<int>(j - 1);
        }
    }

    // Filter by gate and original matrix dimensions.
    for (size_t r = 0; r < n_rows; ++r) {
        const int c = row_to_col[r];
        if (c >= 0 && static_cast<size_t>(c) < n_cols) {
            const double original_cost = cost_matrix[r * n_cols + static_cast<size_t>(c)];
            if (original_cost <= gate) {
                assignments.push_back({r, static_cast<size_t>(c)});
                continue;
            }
        }
        unmatched_rows.push_back(r);
    }

    // Find unmatched columns.
    std::vector<bool> col_assigned(n_cols, false);
    for (const auto& a : assignments) col_assigned[a.col] = true;
    for (size_t c = 0; c < n_cols; ++c) {
        if (!col_assigned[c]) unmatched_cols.push_back(c);
    }
}

} // namespace thunderbird::perception::detail
