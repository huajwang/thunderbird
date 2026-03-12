// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — LiDAR-Camera Extrinsic Auto-Calibration Implementation
// ─────────────────────────────────────────────────────────────────────────────
//
// Multi-stage random search for maximizing LiDAR→image edge alignment.
//
// ─────────────────────────────────────────────────────────────────────────────
#include "lidar_camera_calib.h"

#include <algorithm>
#include <cmath>
#include <random>

namespace thunderbird::calib {
namespace {

// ─── Euler (ZYX) → rotation matrix (row-major 3×3) ─────────────────────────

void eulerToRotation(double roll, double pitch, double yaw, double R[9]) {
    double cr = std::cos(roll),  sr = std::sin(roll);
    double cp = std::cos(pitch), sp = std::sin(pitch);
    double cy = std::cos(yaw),   sy = std::sin(yaw);

    R[0] = cy*cp;             R[1] = cy*sp*sr - sy*cr;  R[2] = cy*sp*cr + sy*sr;
    R[3] = sy*cp;             R[4] = sy*sp*sr + cy*cr;  R[5] = sy*sp*cr - cy*sr;
    R[6] = -sp;               R[7] = cp*sr;             R[8] = cp*cr;
}

// ─── Quaternion → rotation matrix ──────────────────────────────────────────

void quatToRotation(const double q[4], double R[9]) {
    double w = q[0], x = q[1], y = q[2], z = q[3];
    R[0] = 1 - 2*(y*y + z*z); R[1] = 2*(x*y - w*z);     R[2] = 2*(x*z + w*y);
    R[3] = 2*(x*y + w*z);     R[4] = 1 - 2*(x*x + z*z); R[5] = 2*(y*z - w*x);
    R[6] = 2*(x*z - w*y);     R[7] = 2*(y*z + w*x);     R[8] = 1 - 2*(x*x + y*y);
}

// ─── Rotation matrix → quaternion [w,x,y,z] ───────────────────────────────

void rotToQuat(const double R[9], double q[4]) {
    double tr = R[0] + R[4] + R[8];
    if (tr > 0) {
        double s = 0.5 / std::sqrt(tr + 1.0);
        q[0] = 0.25 / s;
        q[1] = (R[7] - R[5]) * s;
        q[2] = (R[2] - R[6]) * s;
        q[3] = (R[3] - R[1]) * s;
    } else if (R[0] > R[4] && R[0] > R[8]) {
        double s = 2.0 * std::sqrt(1.0 + R[0] - R[4] - R[8]);
        q[0] = (R[7] - R[5]) / s; q[1] = 0.25 * s;
        q[2] = (R[1] + R[3]) / s; q[3] = (R[2] + R[6]) / s;
    } else if (R[4] > R[8]) {
        double s = 2.0 * std::sqrt(1.0 + R[4] - R[0] - R[8]);
        q[0] = (R[2] - R[6]) / s; q[1] = (R[1] + R[3]) / s;
        q[2] = 0.25 * s;          q[3] = (R[5] + R[7]) / s;
    } else {
        double s = 2.0 * std::sqrt(1.0 + R[8] - R[0] - R[4]);
        q[0] = (R[3] - R[1]) / s; q[1] = (R[2] + R[6]) / s;
        q[2] = (R[5] + R[7]) / s; q[3] = 0.25 * s;
    }
    double n = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n > 0) { q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n; }
    if (q[0] < 0) { q[0] = -q[0]; q[1] = -q[1]; q[2] = -q[2]; q[3] = -q[3]; }
}

// ─── 3×3 matrix multiply ──────────────────────────────────────────────────

void mat3_mul(const double A[9], const double B[9], double C[9]) {
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) {
            C[r*3+c] = 0;
            for (int k = 0; k < 3; ++k)
                C[r*3+c] += A[r*3+k] * B[k*3+c];
        }
}

// ─── Transform a 3D point: p_cam = R * p_lidar + t ────────────────────────

struct Point2D { double x, y; };

bool projectPoint(const CalibPoint3D& pt,
                  const double R[9], const double t[3],
                  const CameraIntrinsics& intr,
                  double min_depth,
                  Point2D& px) {
    // Transform to camera frame
    double cx = R[0]*pt.x + R[1]*pt.y + R[2]*pt.z + t[0];
    double cy = R[3]*pt.x + R[4]*pt.y + R[5]*pt.z + t[1];
    double cz = R[6]*pt.x + R[7]*pt.y + R[8]*pt.z + t[2];

    if (cz < min_depth) return false;

    // Pinhole projection (no distortion — assume pre-undistorted image)
    double inv_z = 1.0 / cz;
    px.x = intr.fx * cx * inv_z + intr.cx;
    px.y = intr.fy * cy * inv_z + intr.cy;

    return (px.x >= 0 && px.x < static_cast<double>(intr.width) &&
            px.y >= 0 && px.y < static_cast<double>(intr.height));
}

// ─── Compute cost: fraction of projected points on edges ───────────────────

double computeCost(const CalibPoint3D* points, int n_points,
                   const GrayImage& edge_img,
                   const CameraIntrinsics& intr,
                   const double R[9], const double t[3],
                   double min_depth, double edge_threshold,
                   int& projected_count, int& edge_count) {
    projected_count = 0;
    edge_count = 0;

    for (int i = 0; i < n_points; ++i) {
        Point2D px;
        if (!projectPoint(points[i], R, t, intr, min_depth, px))
            continue;

        int ix = static_cast<int>(px.x + 0.5);
        int iy = static_cast<int>(px.y + 0.5);
        if (ix < 0 || ix >= edge_img.width || iy < 0 || iy >= edge_img.height)
            continue;

        ++projected_count;

        double val = static_cast<double>(edge_img.data[iy * edge_img.width + ix]);
        if (val > edge_threshold)
            ++edge_count;
    }

    if (projected_count == 0) return 0.0;
    return static_cast<double>(edge_count) / static_cast<double>(projected_count);
}

// ─── Apply a 6-DOF delta to the current R, t ──────────────────────────────

void applyDelta(const double R_base[9], const double t_base[3],
                double dr, double dp, double dy,
                double dx, double ddy, double dz,
                double R_out[9], double t_out[3]) {
    // Delta rotation from Euler angles
    double dR[9];
    eulerToRotation(dr, dp, dy, dR);

    // R_new = R_base × dR  (compose in body frame)
    mat3_mul(R_base, dR, R_out);

    // t_new = t_base + R_base × [dx, dy, dz]  (delta in body frame)
    t_out[0] = t_base[0] + R_base[0]*dx + R_base[1]*ddy + R_base[2]*dz;
    t_out[1] = t_base[1] + R_base[3]*dx + R_base[4]*ddy + R_base[5]*dz;
    t_out[2] = t_base[2] + R_base[6]*dx + R_base[7]*ddy + R_base[8]*dz;
}

}  // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────

LidarCameraCalibResult calibrateLidarCameraWithProgress(
    const CalibPoint3D* points, int n_points,
    const GrayImage& edge_image,
    const CameraIntrinsics& intrinsics,
    const SensorExtrinsic& initial,
    const LidarCameraCalibConfig& config,
    CalibProgressCallback progress) {

    LidarCameraCalibResult result;
    if (!points || n_points == 0 || !edge_image.data) return result;

    // Convert initial extrinsic to rotation matrix + translation
    double R_best[9], t_best[3];
    quatToRotation(initial.rotation.data(), R_best);
    t_best[0] = initial.translation[0];
    t_best[1] = initial.translation[1];
    t_best[2] = initial.translation[2];

    const double deg2rad = 3.14159265358979323846 / 180.0;

    // Evaluate initial score
    int proj_count, edge_count;
    double best_score = computeCost(points, n_points, edge_image, intrinsics,
                                    R_best, t_best, config.min_depth,
                                    config.edge_threshold,
                                    proj_count, edge_count);

    std::mt19937 rng(42);

    double rot_range = config.init_rot_range_deg * deg2rad;
    double trans_range = config.init_trans_range_m;

    for (int stage = 0; stage < config.num_stages; ++stage) {
        std::uniform_real_distribution<double> rot_dist(-rot_range, rot_range);
        std::uniform_real_distribution<double> trans_dist(-trans_range, trans_range);

        bool improved = false;

        for (int s = 0; s < config.samples_per_stage; ++s) {
            double dr = rot_dist(rng);
            double dp = rot_dist(rng);
            double dy = rot_dist(rng);
            double dx = trans_dist(rng);
            double ddy = trans_dist(rng);
            double dz = trans_dist(rng);

            double R_cand[9], t_cand[3];
            applyDelta(R_best, t_best, dr, dp, dy, dx, ddy, dz, R_cand, t_cand);

            int p_cnt, e_cnt;
            double score = computeCost(points, n_points, edge_image, intrinsics,
                                       R_cand, t_cand, config.min_depth,
                                       config.edge_threshold, p_cnt, e_cnt);

            if (score > best_score) {
                best_score = score;
                std::copy(R_cand, R_cand + 9, R_best);
                std::copy(t_cand, t_cand + 3, t_best);
                proj_count = p_cnt;
                edge_count = e_cnt;
                improved = true;
            }
        }

        if (progress) progress(stage, config.samples_per_stage, best_score);

        // Narrow search range
        rot_range *= config.range_decay;
        trans_range *= config.range_decay;

        // If no improvement, further narrow
        if (!improved) {
            rot_range *= config.range_decay;
            trans_range *= config.range_decay;
        }
    }

    // Convert back to quaternion
    double q[4];
    rotToQuat(R_best, q);

    result.camera_T_lidar.rotation[0] = q[0];
    result.camera_T_lidar.rotation[1] = q[1];
    result.camera_T_lidar.rotation[2] = q[2];
    result.camera_T_lidar.rotation[3] = q[3];
    result.camera_T_lidar.translation[0] = t_best[0];
    result.camera_T_lidar.translation[1] = t_best[1];
    result.camera_T_lidar.translation[2] = t_best[2];
    result.score = best_score;
    result.total_projected = proj_count;
    result.edge_matches = edge_count;
    result.converged = (best_score > 0.0);

    return result;
}

LidarCameraCalibResult calibrateLidarCamera(
    const CalibPoint3D* points, int n_points,
    const GrayImage& edge_image,
    const CameraIntrinsics& intrinsics,
    const SensorExtrinsic& initial,
    const LidarCameraCalibConfig& config) {
    return calibrateLidarCameraWithProgress(
        points, n_points, edge_image, intrinsics, initial, config, nullptr);
}

}  // namespace thunderbird::calib
