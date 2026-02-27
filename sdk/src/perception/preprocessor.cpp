// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Point cloud preprocessor implementation
// ─────────────────────────────────────────────────────────────────────────────
//
// Pipeline: ROI crop → voxel downsample → ground removal → clustering.
// All CPU, no GPU dependencies.
//
// Spatial indexing for clustering uses a simple voxel hash grid —
// no external library needed.  For production-scale clouds (>200 k pts)
// consider replacing with a nanoflann KD-tree for the clustering step.
//
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/perception/preprocessor.h"
#include "thunderbird/odom/slam_types.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <queue>
#include <unordered_map>
#include <vector>

namespace thunderbird::perception {

// ═════════════════════════════════════════════════════════════════════════════
//  Voxel hash for downsampling and spatial queries
// ═════════════════════════════════════════════════════════════════════════════

namespace {

struct VoxelKey {
    int32_t x, y, z;
    bool operator==(const VoxelKey& o) const noexcept {
        return x == o.x && y == o.y && z == o.z;
    }
};

struct VoxelKeyHash {
    size_t operator()(const VoxelKey& k) const noexcept {
        // Fast spatial hash — good distribution for uniform grids.
        auto h = static_cast<size_t>(k.x * 73856093);
        h ^= static_cast<size_t>(k.y * 19349663);
        h ^= static_cast<size_t>(k.z * 83492791);
        return h;
    }
};

inline VoxelKey point_to_voxel(float px, float py, float pz, double inv_size) noexcept {
    return {
        static_cast<int32_t>(std::floor(px * inv_size)),
        static_cast<int32_t>(std::floor(py * inv_size)),
        static_cast<int32_t>(std::floor(pz * inv_size)),
    };
}

} // anonymous namespace

// ═════════════════════════════════════════════════════════════════════════════
//  PImpl
// ═════════════════════════════════════════════════════════════════════════════

struct PointCloudPreprocessor::Impl {
    PreprocessorConfig config;

    // Pre-computed constants.
    double roi_radius_sq{0};
    double inv_voxel_size{0};
    double inv_cluster_eps{0};

    // Reusable scratch buffers (avoid per-frame allocation).
    std::vector<odom::PointXYZIT> roi_points;
    std::vector<odom::PointXYZIT> downsampled;
    std::vector<odom::PointXYZIT> non_ground;
    std::vector<uint32_t>         labels;

    explicit Impl(const PreprocessorConfig& cfg)
        : config(cfg)
        , roi_radius_sq(cfg.roi_radius * cfg.roi_radius)
        , inv_voxel_size(1.0 / std::max(cfg.voxel_size, 0.01))
        , inv_cluster_eps(1.0 / std::max(cfg.cluster_eps, 0.01))
    {}

    // ── Step 1: ROI crop ────────────────────────────────────────────────

    void roiCrop(const odom::PointCloudFrame& cloud) {
        roi_points.clear();
        roi_points.reserve(cloud.points.size());

        const double r2 = roi_radius_sq;
        const float  zmin = static_cast<float>(config.roi_z_min);
        const float  zmax = static_cast<float>(config.roi_z_max);

        for (const auto& p : cloud.points) {
            const double d2 = static_cast<double>(p.x) * p.x +
                              static_cast<double>(p.y) * p.y;
            if (d2 <= r2 && p.z >= zmin && p.z <= zmax) {
                roi_points.push_back(p);
            }
        }
    }

    // ── Step 2: Voxel downsampling ──────────────────────────────────────

    void voxelDownsample() {
        // Hash each point into a voxel; keep the centroid.
        struct VoxelAccum {
            double sx{0}, sy{0}, sz{0}, si{0};
            int count{0};
        };

        std::unordered_map<VoxelKey, VoxelAccum, VoxelKeyHash> grid;
        grid.reserve(roi_points.size() / 4);

        for (const auto& p : roi_points) {
            auto key = point_to_voxel(p.x, p.y, p.z, inv_voxel_size);
            auto& v = grid[key];
            v.sx += p.x; v.sy += p.y; v.sz += p.z;
            v.si += p.intensity;
            v.count++;
        }

        downsampled.clear();
        downsampled.reserve(grid.size());
        for (const auto& [key, v] : grid) {
            const double inv = 1.0 / v.count;
            odom::PointXYZIT p;
            p.x = static_cast<float>(v.sx * inv);
            p.y = static_cast<float>(v.sy * inv);
            p.z = static_cast<float>(v.sz * inv);
            p.intensity = static_cast<float>(v.si * inv);
            p.dt_ns = 0;
            downsampled.push_back(p);
        }
    }

    // ── Step 3: Ground removal ──────────────────────────────────────────

    void removeGround(double ground_plane[4]) {
        ground_plane[0] = ground_plane[1] = ground_plane[2] = ground_plane[3] = 0;
        non_ground.clear();
        non_ground.reserve(downsampled.size());

        if (config.ground_method == GroundRemovalMethod::HeightThreshold) {
            // Simple height-based removal.
            const float thresh = static_cast<float>(config.ground_height_threshold);
            ground_plane[2] = 1.0;  // z-up plane
            ground_plane[3] = -config.ground_height_threshold;

            for (const auto& p : downsampled) {
                if (p.z > thresh) {
                    non_ground.push_back(p);
                }
            }
        } else {
            // RANSAC plane fitting.
            ransacGroundPlane(ground_plane);
        }
    }

    void ransacGroundPlane(double plane[4]) {
        // Simplified RANSAC: fit a horizontal-ish plane.
        if (downsampled.size() < 3) {
            non_ground = downsampled;
            return;
        }

        const int max_iter = config.ransac_max_iterations;
        const double dist_thresh = config.ransac_distance_threshold;
        const size_t n = downsampled.size();

        double best_plane[4]{0, 0, 1, 0};
        size_t best_inliers = 0;

        // Simple LCG for deterministic pseudo-random sampling.
        uint32_t rng_state = 42;
        auto rng_next = [&]() -> uint32_t {
            rng_state = rng_state * 1664525u + 1013904223u;
            return rng_state;
        };

        for (int iter = 0; iter < max_iter; ++iter) {
            // Pick 3 random points.
            const auto& p0 = downsampled[rng_next() % n];
            const auto& p1 = downsampled[rng_next() % n];
            const auto& p2 = downsampled[rng_next() % n];

            // Cross product: normal = (p1-p0) × (p2-p0)
            const double ux = p1.x - p0.x, uy = p1.y - p0.y, uz = p1.z - p0.z;
            const double vx = p2.x - p0.x, vy = p2.y - p0.y, vz = p2.z - p0.z;
            double nx = uy * vz - uz * vy;
            double ny = uz * vx - ux * vz;
            double nz = ux * vy - uy * vx;

            const double len = std::sqrt(nx*nx + ny*ny + nz*nz);
            if (len < 1e-9) continue;
            nx /= len; ny /= len; nz /= len;

            // Reject planes that are not approximately horizontal (z-up).
            if (std::abs(nz) < 0.7) continue;

            const double d = -(nx * p0.x + ny * p0.y + nz * p0.z);

            // Count inliers.
            size_t inliers = 0;
            for (const auto& p : downsampled) {
                const double dist = std::abs(nx * p.x + ny * p.y + nz * p.z + d);
                if (dist < dist_thresh) ++inliers;
            }

            if (inliers > best_inliers) {
                best_inliers = inliers;
                best_plane[0] = nx;
                best_plane[1] = ny;
                best_plane[2] = nz;
                best_plane[3] = d;
            }
        }

        plane[0] = best_plane[0];
        plane[1] = best_plane[1];
        plane[2] = best_plane[2];
        plane[3] = best_plane[3];

        // Separate ground and non-ground.
        for (const auto& p : downsampled) {
            const double dist = std::abs(
                best_plane[0] * p.x + best_plane[1] * p.y +
                best_plane[2] * p.z + best_plane[3]);
            if (dist >= dist_thresh) {
                non_ground.push_back(p);
            }
        }
    }

    // ── Step 4: Euclidean clustering ────────────────────────────────────

    uint32_t euclideanCluster() {
        const size_t n = non_ground.size();
        labels.assign(n, 0);  // 0 = unassigned

        if (n == 0) return 0;

        // Build a voxel grid for O(1) neighbor lookup.
        std::unordered_map<VoxelKey, std::vector<uint32_t>, VoxelKeyHash> spatial;
        spatial.reserve(n);
        for (uint32_t i = 0; i < n; ++i) {
            auto key = point_to_voxel(
                non_ground[i].x, non_ground[i].y, non_ground[i].z,
                inv_cluster_eps);
            spatial[key].push_back(i);
        }

        uint32_t cluster_id = 0;
        std::queue<uint32_t> frontier;

        for (uint32_t seed = 0; seed < n; ++seed) {
            if (labels[seed] != 0) continue;
            if (cluster_id >= static_cast<uint32_t>(config.max_clusters)) break;

            ++cluster_id;
            int cluster_size = 0;
            frontier.push(seed);
            labels[seed] = cluster_id;

            while (!frontier.empty()) {
                const uint32_t idx = frontier.front();
                frontier.pop();
                ++cluster_size;

                if (cluster_size > config.cluster_max_points) break;

                const auto& p = non_ground[idx];
                auto center_key = point_to_voxel(
                    p.x, p.y, p.z, inv_cluster_eps);

                // Search the 3×3×3 neighbor voxels.
                for (int dx = -1; dx <= 1; ++dx) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dz = -1; dz <= 1; ++dz) {
                            VoxelKey nk{center_key.x + dx, center_key.y + dy,
                                        center_key.z + dz};
                            auto it = spatial.find(nk);
                            if (it == spatial.end()) continue;

                            for (uint32_t ni : it->second) {
                                if (labels[ni] != 0) continue;
                                const auto& np = non_ground[ni];
                                const float ddx = p.x - np.x;
                                const float ddy = p.y - np.y;
                                const float ddz = p.z - np.z;
                                const double d2 = static_cast<double>(ddx)*ddx +
                                                  static_cast<double>(ddy)*ddy +
                                                  static_cast<double>(ddz)*ddz;
                                if (d2 <= config.cluster_eps * config.cluster_eps) {
                                    labels[ni] = cluster_id;
                                    frontier.push(ni);
                                }
                            }
                        }
                    }
                }
            }

            // Remove too-small clusters; relabel to 0 (unassigned).
            if (cluster_size < config.cluster_min_points) {
                for (uint32_t i = 0; i < n; ++i) {
                    if (labels[i] == cluster_id) labels[i] = 0;
                }
                --cluster_id;
            }
        }

        return cluster_id;
    }
};

// ═════════════════════════════════════════════════════════════════════════════
//  Public wrapper
// ═════════════════════════════════════════════════════════════════════════════

PointCloudPreprocessor::PointCloudPreprocessor(const PreprocessorConfig& config)
    : pimpl_(std::make_unique<Impl>(config)) {}

PointCloudPreprocessor::~PointCloudPreprocessor() = default;

DetectionInput PointCloudPreprocessor::process(
    std::shared_ptr<const odom::PointCloudFrame> cloud,
    const odom::Pose6D& ego_pose,
    int64_t timestamp_ns)
{
    DetectionInput result;
    result.timestamp_ns = timestamp_ns;
    result.ego_pose     = ego_pose;

    if (!cloud || cloud->points.empty()) {
        result.raw_point_count = 0;
        return result;
    }

    result.raw_point_count = cloud->num_points();

    // Pipeline steps.
    pimpl_->roiCrop(*cloud);
    pimpl_->voxelDownsample();
    pimpl_->removeGround(result.ground_plane);
    result.num_clusters = pimpl_->euclideanCluster();

    // Build the filtered cloud from non-ground points.
    auto filtered = std::make_shared<odom::PointCloudFrame>();
    filtered->timestamp_ns = timestamp_ns;
    filtered->sequence     = cloud->sequence;
    filtered->is_deskewed  = cloud->is_deskewed;
    filtered->points       = pimpl_->non_ground;  // copy — could move if needed

    result.filtered_cloud  = std::move(filtered);
    result.cluster_labels  = pimpl_->labels;       // copy

    return result;
}

} // namespace thunderbird::perception
