// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — 3D RANSAC Line Fitting
// ─────────────────────────────────────────────────────────────────────────────
//
// Standard RANSAC-based 3D line fitting (no external dependencies).
//
// Given a set of 3D points, fits a line L(t) = origin + t * direction
// using RANSAC to robustly handle outliers.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <cmath>
#include <cstddef>
#include <random>
#include <vector>

namespace thunderbird::calib {

/// Result of 3D line fitting.
struct LineFitResult {
    double origin[3] = {};     ///< A point on the line (centroid of inliers).
    double direction[3] = {};  ///< Unit direction vector.
    double rmse = 0.0;         ///< RMS distance of inliers to the line.
    std::vector<int> inliers;  ///< Indices of inlier points.
    bool valid = false;
};

/// Fit a 3D line using RANSAC.
///
/// @param points     Flat array [x0,y0,z0, x1,y1,z1, ...].
/// @param n_points   Number of points.
/// @param threshold  Maximum point-to-line distance for inlier (meters).
/// @param max_iter   Maximum RANSAC iterations.
/// @return LineFitResult with inlier indices and line parameters.
inline LineFitResult fitLine3D(const double* points, int n_points,
                               double threshold = 0.02, int max_iter = 100) {
    LineFitResult best;
    if (!points || n_points < 2) return best;

    // For 2 points, direct solution
    if (n_points == 2) {
        for (int j = 0; j < 3; ++j) {
            best.origin[j] = 0.5 * (points[j] + points[3 + j]);
            best.direction[j] = points[3 + j] - points[j];
        }
        double len = std::sqrt(best.direction[0]*best.direction[0] +
                               best.direction[1]*best.direction[1] +
                               best.direction[2]*best.direction[2]);
        if (len > 1e-12) {
            best.direction[0] /= len; best.direction[1] /= len; best.direction[2] /= len;
        }
        best.inliers = {0, 1};
        best.rmse = 0.0;
        best.valid = true;
        return best;
    }

    std::mt19937 rng(42);
    int best_count = 0;

    for (int iter = 0; iter < max_iter; ++iter) {
        // Sample 2 random points
        int i0 = static_cast<int>(rng() % static_cast<unsigned>(n_points));
        int i1;
        do { i1 = static_cast<int>(rng() % static_cast<unsigned>(n_points)); }
        while (i1 == i0);

        // Line direction
        double d[3];
        for (int j = 0; j < 3; ++j)
            d[j] = points[i1 * 3 + j] - points[i0 * 3 + j];
        double len = std::sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2]);
        if (len < 1e-12) continue;
        d[0] /= len; d[1] /= len; d[2] /= len;

        // Count inliers: distance from point to line
        std::vector<int> inliers;
        for (int i = 0; i < n_points; ++i) {
            // Vector from line point to test point
            double v[3];
            for (int j = 0; j < 3; ++j)
                v[j] = points[i * 3 + j] - points[i0 * 3 + j];

            // Project v onto d
            double proj = v[0]*d[0] + v[1]*d[1] + v[2]*d[2];

            // Perpendicular distance = |v - proj*d|
            double perp[3];
            for (int j = 0; j < 3; ++j)
                perp[j] = v[j] - proj * d[j];
            double dist = std::sqrt(perp[0]*perp[0] + perp[1]*perp[1] + perp[2]*perp[2]);

            if (dist < threshold)
                inliers.push_back(i);
        }

        if (static_cast<int>(inliers.size()) > best_count) {
            best_count = static_cast<int>(inliers.size());

            // Refit using inlier centroid and SVD-free PCA (power iteration)
            double centroid[3] = {};
            for (int idx : inliers)
                for (int j = 0; j < 3; ++j)
                    centroid[j] += points[idx * 3 + j];
            double inv_n = 1.0 / static_cast<double>(inliers.size());
            for (int j = 0; j < 3; ++j)
                centroid[j] *= inv_n;

            // Covariance matrix (3×3 symmetric) for direction estimation
            double C[9] = {};
            for (int idx : inliers) {
                double p[3];
                for (int j = 0; j < 3; ++j)
                    p[j] = points[idx * 3 + j] - centroid[j];
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c)
                        C[r * 3 + c] += p[r] * p[c];
            }

            // Power iteration to find dominant eigenvector
            double ev[3] = {d[0], d[1], d[2]};  // Initialize with RANSAC direction
            for (int pi = 0; pi < 20; ++pi) {
                double nv[3] = {};
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c)
                        nv[r] += C[r * 3 + c] * ev[c];
                double nm = std::sqrt(nv[0]*nv[0] + nv[1]*nv[1] + nv[2]*nv[2]);
                if (nm < 1e-12) break;
                ev[0] = nv[0] / nm; ev[1] = nv[1] / nm; ev[2] = nv[2] / nm;
            }

            // Compute RMSE
            double sse = 0.0;
            for (int idx : inliers) {
                double v[3];
                for (int j = 0; j < 3; ++j)
                    v[j] = points[idx * 3 + j] - centroid[j];
                double proj = v[0]*ev[0] + v[1]*ev[1] + v[2]*ev[2];
                double perp_sq = 0.0;
                for (int j = 0; j < 3; ++j) {
                    double p = v[j] - proj * ev[j];
                    perp_sq += p * p;
                }
                sse += perp_sq;
            }

            for (int j = 0; j < 3; ++j) {
                best.origin[j] = centroid[j];
                best.direction[j] = ev[j];
            }
            best.inliers = inliers;
            best.rmse = std::sqrt(sse / static_cast<double>(inliers.size()));
            best.valid = true;
        }
    }

    return best;
}

}  // namespace thunderbird::calib
