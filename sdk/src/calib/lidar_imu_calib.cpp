// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — BALM LiDAR-IMU Calibration Implementation
// ─────────────────────────────────────────────────────────────────────────────
//
// When THUNDERBIRD_HAS_CERES is defined, provides the full BALM optimization.
// Otherwise, stub functions return failure so non-Ceres builds still compile.
//
// ─────────────────────────────────────────────────────────────────────────────
#include "lidar_imu_calib.h"

#ifdef THUNDERBIRD_HAS_CERES

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <numeric>

namespace thunderbird::calib {
namespace {

// ─── Voxel hash key ────────────────────────────────────────────────────────

struct VoxelKey {
    int64_t x, y, z;
    bool operator==(const VoxelKey& o) const { return x == o.x && y == o.y && z == o.z; }
};

struct VoxelKeyHash {
    size_t operator()(const VoxelKey& k) const {
        // FNV-1a hash
        size_t h = 14695981039346656037ULL;
        h ^= static_cast<size_t>(k.x); h *= 1099511628211ULL;
        h ^= static_cast<size_t>(k.y); h *= 1099511628211ULL;
        h ^= static_cast<size_t>(k.z); h *= 1099511628211ULL;
        return h;
    }
};

// ─── Voxel feature data ────────────────────────────────────────────────────

struct VoxelFeature {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();  // Smallest eigenvector
    double eigen_ratio = 0.0;
    int point_count = 0;
    bool is_valid = false;
};

// ─── Compute voxel features (PCA via eigendecomposition) ───────────────────

VoxelFeature computeVoxelFeature(const std::vector<Eigen::Vector3d>& points,
                                  double surface_ratio, double corner_ratio) {
    VoxelFeature feat;
    int n = static_cast<int>(points.size());
    if (n < 5) return feat;

    // Compute centroid
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto& p : points) centroid += p;
    centroid /= static_cast<double>(n);

    // Covariance matrix
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : points) {
        Eigen::Vector3d d = p - centroid;
        cov += d * d.transpose();
    }
    cov /= static_cast<double>(n);

    // Eigendecomposition
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    if (solver.info() != Eigen::Success) return feat;

    Eigen::Vector3d eigenvalues = solver.eigenvalues();
    Eigen::Matrix3d eigenvectors = solver.eigenvectors();

    // Eigenvalues are sorted ascending by Eigen
    // For surface: λ₂/λ₁ >> 1 (smallest eigenvalue is much smaller)
    // For corner: λ₃/λ₂ >> 1 (largest eigenvalue dominates)
    if (eigenvalues(0) < 1e-10) return feat;

    if (eigenvalues(1) < 1e-10) return feat;

    double ratio = eigenvalues(2) / eigenvalues(0);
    double corner_spread = eigenvalues(2) / eigenvalues(1);

    // Accept features with sufficient eigenvalue spread
    if (ratio > surface_ratio || corner_spread > corner_ratio) {
        feat.center = centroid;
        feat.normal = eigenvectors.col(0);  // Smallest eigenvector = surface normal
        feat.eigen_ratio = ratio;
        feat.point_count = n;
        feat.is_valid = true;
    }

    return feat;
}

// ─── Ceres cost function: surface feature alignment ────────────────────────

struct BalmSurfaceCost {
    Eigen::Vector3d point;     // Point in LiDAR frame
    Eigen::Vector3d center;    // Voxel center in world frame
    Eigen::Vector3d normal;    // Surface normal in world frame
    Eigen::Matrix4d T_world_imu;  // IMU pose (world ← IMU)

    BalmSurfaceCost(const Eigen::Vector3d& p, const Eigen::Vector3d& c,
                    const Eigen::Vector3d& n, const Eigen::Matrix4d& T)
        : point(p), center(c), normal(n), T_world_imu(T) {}

    template <typename T>
    bool operator()(const T* const q_imu_lidar,  // [w,x,y,z]
                    const T* const t_imu_lidar,  // [x,y,z]
                    T* residual) const {
        // Transform point: LiDAR → IMU → world
        // p_imu = R_imu_lidar * p_lidar + t_imu_lidar
        T p_lidar[3] = {T(point.x()), T(point.y()), T(point.z())};
        T p_imu[3];

        // Ceres quaternion convention: [w,x,y,z]
        ceres::QuaternionRotatePoint(q_imu_lidar, p_lidar, p_imu);
        p_imu[0] += t_imu_lidar[0];
        p_imu[1] += t_imu_lidar[1];
        p_imu[2] += t_imu_lidar[2];

        // p_world = T_world_imu * p_imu
        T p_world[3];
        for (int i = 0; i < 3; ++i) {
            p_world[i] = T(T_world_imu(i, 0)) * p_imu[0]
                        + T(T_world_imu(i, 1)) * p_imu[1]
                        + T(T_world_imu(i, 2)) * p_imu[2]
                        + T(T_world_imu(i, 3));
        }

        // Residual: distance from point to surface plane
        // r = normal · (p_world - center)
        residual[0] = T(normal.x()) * (p_world[0] - T(center.x()))
                    + T(normal.y()) * (p_world[1] - T(center.y()))
                    + T(normal.z()) * (p_world[2] - T(center.z()));

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& p,
                                        const Eigen::Vector3d& c,
                                        const Eigen::Vector3d& n,
                                        const Eigen::Matrix4d& T) {
        return new ceres::AutoDiffCostFunction<BalmSurfaceCost, 1, 4, 3>(
            new BalmSurfaceCost(p, c, n, T));
    }
};

// ─── Build voxel map from transformed points ───────────────────────────────

using VoxelMap = std::unordered_map<VoxelKey, std::vector<Eigen::Vector3d>, VoxelKeyHash>;

void buildVoxelMap(const std::vector<Eigen::Vector3d>& world_points,
                   double voxel_size, VoxelMap& voxels) {
    double inv_size = 1.0 / voxel_size;
    for (const auto& p : world_points) {
        VoxelKey key;
        key.x = static_cast<int64_t>(std::floor(p.x() * inv_size));
        key.y = static_cast<int64_t>(std::floor(p.y() * inv_size));
        key.z = static_cast<int64_t>(std::floor(p.z() * inv_size));
        voxels[key].push_back(p);
    }
}

// ─── Convert SensorExtrinsic to Eigen ──────────────────────────────────────

Eigen::Matrix4d extrinsicToEigen(const SensorExtrinsic& ext) {
    Eigen::Quaterniond q(ext.rotation[0], ext.rotation[1],
                         ext.rotation[2], ext.rotation[3]);
    q.normalize();
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = q.toRotationMatrix();
    T(0,3) = ext.translation[0];
    T(1,3) = ext.translation[1];
    T(2,3) = ext.translation[2];
    return T;
}

Eigen::Matrix4d poseToEigen(const double rot[4], const double trans[3]) {
    Eigen::Quaterniond q(rot[0], rot[1], rot[2], rot[3]);
    q.normalize();
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = q.toRotationMatrix();
    T(0,3) = trans[0]; T(1,3) = trans[1]; T(2,3) = trans[2];
    return T;
}

// ─── Downsample points in a voxel grid ─────────────────────────────────────

void downsamplePoints(std::vector<Eigen::Vector3d>& points, double leaf_size) {
    if (leaf_size <= 0 || points.empty()) return;

    std::unordered_map<VoxelKey, Eigen::Vector3d, VoxelKeyHash> grid;
    std::unordered_map<VoxelKey, int, VoxelKeyHash> counts;
    double inv = 1.0 / leaf_size;

    for (const auto& p : points) {
        VoxelKey k;
        k.x = static_cast<int64_t>(std::floor(p.x() * inv));
        k.y = static_cast<int64_t>(std::floor(p.y() * inv));
        k.z = static_cast<int64_t>(std::floor(p.z() * inv));
        auto& sum = grid[k];
        if (counts[k] == 0)
            sum = Eigen::Vector3d::Zero();
        sum += p;
        counts[k]++;
    }

    points.clear();
    points.reserve(grid.size());
    for (auto& [key, sum] : grid) {
        points.push_back(sum / static_cast<double>(counts[key]));
    }
}

}  // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────

bool isLidarImuCalibAvailable() { return true; }

LidarImuCalibResult calibrateLidarImuWithProgress(
    const std::vector<CalibFrame>& frames,
    const SensorExtrinsic& initial,
    const LidarImuCalibConfig& config,
    LidarImuProgressCallback progress) {

    LidarImuCalibResult result;
    if (frames.size() < 3) return result;

    // Initialize parameters: quaternion [w,x,y,z] and translation [x,y,z]
    double q[4] = { initial.rotation[0], initial.rotation[1],
                     initial.rotation[2], initial.rotation[3] };
    double t[3] = { initial.translation[0], initial.translation[1],
                     initial.translation[2] };

    // Normalize quaternion
    double qn = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (qn > 0) { q[0] /= qn; q[1] /= qn; q[2] /= qn; q[3] /= qn; }

    // Pre-compute IMU poses as Eigen matrices
    std::vector<Eigen::Matrix4d> imu_poses;
    imu_poses.reserve(frames.size());
    for (const auto& frame : frames) {
        imu_poses.push_back(poseToEigen(frame.pose_rotation, frame.pose_translation));
    }

    double last_cost = std::numeric_limits<double>::max();

    for (int round = 0; round < config.num_rounds; ++round) {
        // Current imu_T_lidar
        Eigen::Matrix4d T_imu_lidar = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond q_current(q[0], q[1], q[2], q[3]);
        q_current.normalize();
        T_imu_lidar.block<3,3>(0,0) = q_current.toRotationMatrix();
        T_imu_lidar(0,3) = t[0]; T_imu_lidar(1,3) = t[1]; T_imu_lidar(2,3) = t[2];

        // Transform all points to world frame and build voxel map
        VoxelMap voxels;

        for (size_t fi = 0; fi < frames.size(); ++fi) {
            const auto& frame = frames[fi];
            Eigen::Matrix4d T_world_lidar = imu_poses[fi] * T_imu_lidar;

            std::vector<Eigen::Vector3d> world_pts;
            world_pts.reserve(frame.points.size());

            for (const auto& pt : frame.points) {
                Eigen::Vector4d p(pt.x, pt.y, pt.z, 1.0);
                Eigen::Vector4d pw = T_world_lidar * p;
                world_pts.emplace_back(pw.head<3>());
            }

            if (config.downsample_size > 0)
                downsamplePoints(world_pts, config.downsample_size);

            double inv_size = 1.0 / config.voxel_size;
            for (size_t pi = 0; pi < world_pts.size(); ++pi) {
                VoxelKey key;
                key.x = static_cast<int64_t>(std::floor(world_pts[pi].x() * inv_size));
                key.y = static_cast<int64_t>(std::floor(world_pts[pi].y() * inv_size));
                key.z = static_cast<int64_t>(std::floor(world_pts[pi].z() * inv_size));
                voxels[key].push_back(world_pts[pi]);
            }
        }

        // Extract features from voxels (computed once per voxel)
        std::unordered_map<VoxelKey, VoxelFeature, VoxelKeyHash> feature_map;
        for (auto& [key, pts] : voxels) {
            if (static_cast<int>(pts.size()) < config.min_points_per_voxel) continue;
            auto feat = computeVoxelFeature(pts, config.surface_eigen_ratio,
                                            config.corner_eigen_ratio);
            if (feat.is_valid) feature_map.emplace(key, std::move(feat));
        }

        if (feature_map.empty()) continue;

        // Build Ceres problem
        ceres::Problem problem;

        // Add quaternion parameterization
        problem.AddParameterBlock(q, 4, new ceres::QuaternionParameterization());
        problem.AddParameterBlock(t, 3);

        int residual_count = 0;

        // For each frame, add residuals for points in valid voxels
        for (size_t fi = 0; fi < frames.size(); ++fi) {
            const auto& frame = frames[fi];

            // Sample points from this frame
            int step = std::max(1, static_cast<int>(frame.points.size()) / 500);

            for (size_t pi = 0; pi < frame.points.size(); pi += static_cast<size_t>(step)) {
                const auto& pt = frame.points[pi];
                Eigen::Vector3d p_lidar(pt.x, pt.y, pt.z);

                // Transform to world to find which voxel it belongs to
                Eigen::Matrix4d T_world_lidar = imu_poses[fi] * T_imu_lidar;
                Eigen::Vector4d pw4 = T_world_lidar * Eigen::Vector4d(pt.x, pt.y, pt.z, 1.0);
                Eigen::Vector3d pw = pw4.head<3>();

                double inv_size = 1.0 / config.voxel_size;
                VoxelKey key;
                key.x = static_cast<int64_t>(std::floor(pw.x() * inv_size));
                key.y = static_cast<int64_t>(std::floor(pw.y() * inv_size));
                key.z = static_cast<int64_t>(std::floor(pw.z() * inv_size));

                // Find matching feature
                // We check if the voxel has a valid feature by looking it up
                // in the features list. For efficiency, build a hash set.
                // For now, iterate (features are typically O(1000)).
                // TODO: Use a hash map for O(1) lookup in production.

                auto fit = feature_map.find(key);
                if (fit == feature_map.end()) continue;
                const auto& feat = fit->second;

                problem.AddResidualBlock(
                    BalmSurfaceCost::Create(p_lidar, feat.center, feat.normal,
                                           imu_poses[fi]),
                    new ceres::HuberLoss(0.1),
                    q, t);
                ++residual_count;
            }
        }

        if (residual_count == 0) continue;

        // Solve
        ceres::Solver::Options options;
        options.max_num_iterations = config.solver_max_iterations;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;
        options.num_threads = 1;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        // Normalize quaternion after optimization
        qn = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        if (qn > 0) { q[0] /= qn; q[1] /= qn; q[2] /= qn; q[3] /= qn; }

        last_cost = summary.final_cost;
        result.num_residuals = residual_count;
        result.rounds_completed = round + 1;
        result.converged = (summary.termination_type == ceres::CONVERGENCE);

        if (progress) progress(round, last_cost);

        // Early termination if converged with low cost
        if (result.converged && last_cost < 1e-6) break;
    }

    // Store result
    result.imu_T_lidar.rotation[0] = q[0];
    result.imu_T_lidar.rotation[1] = q[1];
    result.imu_T_lidar.rotation[2] = q[2];
    result.imu_T_lidar.rotation[3] = q[3];
    result.imu_T_lidar.translation[0] = t[0];
    result.imu_T_lidar.translation[1] = t[1];
    result.imu_T_lidar.translation[2] = t[2];
    result.final_cost = last_cost;

    return result;
}

LidarImuCalibResult calibrateLidarImu(
    const std::vector<CalibFrame>& frames,
    const SensorExtrinsic& initial,
    const LidarImuCalibConfig& config) {
    return calibrateLidarImuWithProgress(frames, initial, config, nullptr);
}

}  // namespace thunderbird::calib

#else  // !THUNDERBIRD_HAS_CERES

namespace thunderbird::calib {

bool isLidarImuCalibAvailable() { return false; }

LidarImuCalibResult calibrateLidarImu(
    const std::vector<CalibFrame>&,
    const SensorExtrinsic&,
    const LidarImuCalibConfig&) {
    return {};
}

LidarImuCalibResult calibrateLidarImuWithProgress(
    const std::vector<CalibFrame>&,
    const SensorExtrinsic&,
    const LidarImuCalibConfig&,
    LidarImuProgressCallback) {
    return {};
}

}  // namespace thunderbird::calib

#endif  // THUNDERBIRD_HAS_CERES
