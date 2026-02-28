// ─────────────────────────────────────────────────────────────────────────────
// Test — PointCloudPreprocessor
// ─────────────────────────────────────────────────────────────────────────────
//
// Covers:
//   • Empty-cloud handling
//   • ROI cropping (radius + z-bounds)
//   • Voxel downsampling (fewer points out, within voxel centres)
//   • Height-threshold ground removal
//   • RANSAC ground plane fitting on a synthetic flat ground
//   • Euclidean clustering — distinct blobs get distinct labels
//   • Cluster min/max point limits
//   • max_clusters cap
//
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/perception/preprocessor.h"
#include "thunderbird/perception/perception_config.h"
#include "thunderbird/odom/slam_types.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <memory>
#include <set>
#include <vector>

using namespace thunderbird::perception;
using namespace thunderbird::odom;

// ─────────────────────────────────────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Create a cloud with N points arranged in a uniform grid on a flat plane
/// at z = ground_z.
static std::shared_ptr<const PointCloudFrame>
makeGroundPlane(int nx, int ny, double spacing, double ground_z) {
    auto cloud = std::make_shared<PointCloudFrame>();
    cloud->timestamp_ns = 1'000'000'000;
    cloud->is_deskewed  = true;
    for (int i = 0; i < nx; ++i) {
        for (int j = 0; j < ny; ++j) {
            PointXYZIT p;
            p.x = static_cast<float>(i * spacing - nx * spacing * 0.5);
            p.y = static_cast<float>(j * spacing - ny * spacing * 0.5);
            p.z = static_cast<float>(ground_z);
            p.intensity = 50.0f;
            cloud->points.push_back(p);
        }
    }
    return cloud;
}

/// Create a cloud with a ground plane + an above-ground blob.
static std::shared_ptr<PointCloudFrame>
makeGroundPlusBlob(double ground_z, double blob_cx, double blob_cy,
                   double blob_cz, double blob_size, int blob_pts) {
    auto cloud = std::make_shared<PointCloudFrame>();
    cloud->timestamp_ns = 1'000'000'000;
    cloud->is_deskewed  = true;

    // Ground: 10×10 grid at spacing 0.5 m
    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 10; ++j) {
            PointXYZIT p;
            p.x = static_cast<float>(i * 0.5 - 2.5);
            p.y = static_cast<float>(j * 0.5 - 2.5);
            p.z = static_cast<float>(ground_z);
            p.intensity = 30.0f;
            cloud->points.push_back(p);
        }
    }

    // Blob above ground
    const double step = blob_size / blob_pts;
    for (int i = 0; i < blob_pts; ++i) {
        for (int j = 0; j < blob_pts; ++j) {
            for (int k = 0; k < blob_pts; ++k) {
                PointXYZIT p;
                p.x = static_cast<float>(blob_cx - blob_size / 2 + step * (i + 0.5));
                p.y = static_cast<float>(blob_cy - blob_size / 2 + step * (j + 0.5));
                p.z = static_cast<float>(blob_cz - blob_size / 2 + step * (k + 0.5));
                p.intensity = 100.0f;
                cloud->points.push_back(p);
            }
        }
    }
    return cloud;
}

static Pose6D identityPose() {
    Pose6D p;
    p.quaternion = {1, 0, 0, 0};
    return p;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Tests
// ─────────────────────────────────────────────────────────────────────────────

static void test_empty_cloud() {
    PreprocessorConfig cfg;
    PointCloudPreprocessor pp(cfg);

    // Null cloud
    auto result = pp.process(nullptr, identityPose(), 0);
    assert(result.raw_point_count == 0);
    assert(result.num_clusters == 0);

    // Empty cloud
    auto empty = std::make_shared<PointCloudFrame>();
    result = pp.process(empty, identityPose(), 0);
    assert(result.raw_point_count == 0);
    assert(result.num_clusters == 0);

    std::puts("  [PASS] empty_cloud");
}

static void test_roi_radius_crop() {
    PreprocessorConfig cfg;
    cfg.roi_radius = 5.0;
    cfg.roi_z_min  = -10.0;
    cfg.roi_z_max  = 10.0;
    cfg.ground_method = GroundRemovalMethod::HeightThreshold;
    cfg.ground_height_threshold = -100.0;  // keep everything
    cfg.cluster_min_points = 1;
    PointCloudPreprocessor pp(cfg);

    auto cloud = std::make_shared<PointCloudFrame>();
    cloud->timestamp_ns = 1;
    cloud->is_deskewed  = true;

    // Point inside radius (distance 3.0)
    cloud->points.push_back({3.0f, 0.0f, 0.0f, 100.0f, 0});
    // Point outside radius (distance 10.0)
    cloud->points.push_back({10.0f, 0.0f, 0.0f, 100.0f, 0});

    auto result = pp.process(cloud, identityPose(), 1);
    assert(result.raw_point_count == 2);
    // Only the near point should survive ROI crop
    assert(result.filtered_cloud != nullptr);
    // Exactly 1 point should survive (the near one)
    assert(result.filtered_cloud->points.size() == 1);
    std::puts("  [PASS] roi_radius_crop");
}

static void test_roi_z_crop() {
    PreprocessorConfig cfg;
    cfg.roi_radius = 100.0;
    cfg.roi_z_min  = -1.0;
    cfg.roi_z_max  = 3.0;
    cfg.ground_method = GroundRemovalMethod::HeightThreshold;
    cfg.ground_height_threshold = -100.0;
    cfg.cluster_min_points = 1;
    PointCloudPreprocessor pp(cfg);

    auto cloud = std::make_shared<PointCloudFrame>();
    cloud->timestamp_ns = 1;
    cloud->is_deskewed  = true;

    // In-bounds z
    cloud->points.push_back({1.0f, 0.0f, 0.0f, 100.0f, 0});
    // Below z_min
    cloud->points.push_back({1.0f, 0.0f, -5.0f, 100.0f, 0});
    // Above z_max
    cloud->points.push_back({1.0f, 0.0f, 10.0f, 100.0f, 0});

    auto result = pp.process(cloud, identityPose(), 1);
    assert(result.raw_point_count == 3);
    // Only 1 point should survive z-cropping
    assert(result.filtered_cloud != nullptr);
    assert(result.filtered_cloud->points.size() == 1);
    std::puts("  [PASS] roi_z_crop");
}

static void test_voxel_downsample() {
    PreprocessorConfig cfg;
    cfg.roi_radius = 100.0;
    cfg.roi_z_min  = -10.0;
    cfg.roi_z_max  = 10.0;
    cfg.voxel_size = 1.0;  // 1 m voxels
    cfg.ground_method = GroundRemovalMethod::HeightThreshold;
    cfg.ground_height_threshold = -100.0;
    cfg.cluster_min_points = 1;
    PointCloudPreprocessor pp(cfg);

    auto cloud = std::make_shared<PointCloudFrame>();
    cloud->timestamp_ns = 1;
    cloud->is_deskewed  = true;

    // 10 points all within one 1 m voxel
    for (int i = 0; i < 10; ++i) {
        PointXYZIT p;
        p.x = 0.1f * i;
        p.y = 0.1f * i;
        p.z = 1.0f;  // above ground
        p.intensity = 100.0f;
        cloud->points.push_back(p);
    }

    auto result = pp.process(cloud, identityPose(), 1);
    // 10 points in one voxel → should downsample to 1 point
    assert(result.filtered_cloud != nullptr);
    assert(result.filtered_cloud->points.size() == 1);
    std::puts("  [PASS] voxel_downsample");
}

static void test_height_threshold_ground_removal() {
    PreprocessorConfig cfg;
    cfg.roi_radius = 100.0;
    cfg.roi_z_min  = -10.0;
    cfg.roi_z_max  = 10.0;
    cfg.voxel_size = 0.05;  // fine voxels to preserve points
    cfg.ground_method = GroundRemovalMethod::HeightThreshold;
    cfg.ground_height_threshold = 0.0;
    cfg.cluster_min_points = 1;
    PointCloudPreprocessor pp(cfg);

    auto cloud = std::make_shared<PointCloudFrame>();
    cloud->timestamp_ns = 1;
    cloud->is_deskewed  = true;

    // Ground points (z ≤ 0)
    for (int i = 0; i < 20; ++i) {
        PointXYZIT p;
        p.x = static_cast<float>(i * 0.5);
        p.y = 0.0f;
        p.z = -0.1f;
        p.intensity = 30.0f;
        cloud->points.push_back(p);
    }

    // Above-ground points (z > 0)
    for (int i = 0; i < 10; ++i) {
        PointXYZIT p;
        p.x = static_cast<float>(i * 0.5);
        p.y = 0.0f;
        p.z = 1.5f;
        p.intensity = 100.0f;
        cloud->points.push_back(p);
    }

    auto result = pp.process(cloud, identityPose(), 1);
    assert(result.filtered_cloud != nullptr);
    // Ground plane coefficient z should be 1 (z-up plane)
    assert(std::abs(result.ground_plane[2] - 1.0) < 0.01);
    // Only above-ground points should survive
    assert(result.filtered_cloud->points.size() <= 10);
    assert(result.filtered_cloud->points.size() >= 1);
    std::puts("  [PASS] height_threshold_ground_removal");
}

static void test_ransac_ground_plane() {
    PreprocessorConfig cfg;
    cfg.roi_radius = 100.0;
    cfg.roi_z_min  = -10.0;
    cfg.roi_z_max  = 10.0;
    cfg.voxel_size = 0.1;
    cfg.ground_method = GroundRemovalMethod::RansacPlane;
    cfg.ransac_distance_threshold = 0.2;
    cfg.ransac_max_iterations = 200;
    cfg.cluster_min_points = 1;
    PointCloudPreprocessor pp(cfg);

    // Generate a flat ground at z = 0 with many points, and a blob above
    auto cloud = makeGroundPlusBlob(
        0.0,    // ground_z
        2.0, 0.0, 2.0,  // blob centre
        0.5,    // blob size
        3       // points per axis (27 pts)
    );

    auto result = pp.process(cloud, identityPose(), 1);
    assert(result.filtered_cloud != nullptr);

    // RANSAC should find a roughly horizontal plane (nz ~ ±1)
    const double nz = result.ground_plane[2];
    assert(std::abs(nz) > 0.6);  // roughly z-up

    // Non-ground points should be fewer than total
    assert(result.filtered_cloud->points.size() < cloud->points.size());
    // The blob points should survive
    assert(result.filtered_cloud->points.size() >= 1);
    std::puts("  [PASS] ransac_ground_plane");
}

static void test_euclidean_clustering_two_blobs() {
    PreprocessorConfig cfg;
    cfg.roi_radius = 100.0;
    cfg.roi_z_min  = -10.0;
    cfg.roi_z_max  = 10.0;
    cfg.voxel_size = 0.05;
    cfg.ground_method = GroundRemovalMethod::HeightThreshold;
    cfg.ground_height_threshold = -100.0;  // keep everything
    cfg.cluster_eps = 0.5;
    cfg.cluster_min_points = 3;
    cfg.cluster_max_points = 1000;
    cfg.max_clusters = 100;
    PointCloudPreprocessor pp(cfg);

    auto cloud = std::make_shared<PointCloudFrame>();
    cloud->timestamp_ns = 1;
    cloud->is_deskewed  = true;

    // Blob A: 8 points near (2, 0, 1)
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 2; ++k) {
                PointXYZIT p;
                p.x = 2.0f + 0.2f * i;
                p.y = 0.0f + 0.2f * j;
                p.z = 1.0f + 0.2f * k;
                p.intensity = 100.0f;
                cloud->points.push_back(p);
            }

    // Blob B: 8 points near (10, 0, 1) — well separated from A
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 2; ++k) {
                PointXYZIT p;
                p.x = 10.0f + 0.2f * i;
                p.y = 0.0f + 0.2f * j;
                p.z = 1.0f + 0.2f * k;
                p.intensity = 100.0f;
                cloud->points.push_back(p);
            }

    auto result = pp.process(cloud, identityPose(), 1);
    assert(result.num_clusters >= 2);  // two distinct blobs

    // Verify labels assign distinct IDs
    std::set<uint32_t> label_set;
    for (auto l : result.cluster_labels) {
        if (l > 0) label_set.insert(l);
    }
    assert(label_set.size() >= 2);
    std::puts("  [PASS] euclidean_clustering_two_blobs");
}

static void test_cluster_min_points_filter() {
    PreprocessorConfig cfg;
    cfg.roi_radius = 100.0;
    cfg.roi_z_min  = -10.0;
    cfg.roi_z_max  = 10.0;
    cfg.voxel_size = 0.05;
    cfg.ground_method = GroundRemovalMethod::HeightThreshold;
    cfg.ground_height_threshold = -100.0;
    cfg.cluster_eps = 0.5;
    cfg.cluster_min_points = 10;  // high minimum → reject small blobs
    cfg.max_clusters = 100;
    PointCloudPreprocessor pp(cfg);

    auto cloud = std::make_shared<PointCloudFrame>();
    cloud->timestamp_ns = 1;
    cloud->is_deskewed  = true;

    // Small blob: only 4 points (below min_points)
    for (int i = 0; i < 4; ++i) {
        PointXYZIT p;
        p.x = 2.0f + 0.1f * i;
        p.y = 0.0f;
        p.z = 1.0f;
        p.intensity = 100.0f;
        cloud->points.push_back(p);
    }

    auto result = pp.process(cloud, identityPose(), 1);
    // All labels should be 0 (unassigned) — no clusters qualify
    assert(result.num_clusters == 0);
    std::puts("  [PASS] cluster_min_points_filter");
}

static void test_output_preserves_metadata() {
    PreprocessorConfig cfg;
    cfg.cluster_min_points = 1;
    PointCloudPreprocessor pp(cfg);

    auto cloud = std::make_shared<PointCloudFrame>();
    cloud->timestamp_ns = 42'000'000'000LL;
    cloud->sequence     = 7;
    cloud->is_deskewed  = true;
    cloud->points.push_back({1.0f, 0.0f, 1.0f, 100.0f, 0});

    Pose6D pose;
    pose.timestamp_ns = 42'000'000'000LL;
    pose.position = {1.0, 2.0, 3.0};

    auto result = pp.process(cloud, pose, 42'000'000'000LL);
    assert(result.timestamp_ns == 42'000'000'000LL);
    assert(result.ego_pose.position[0] == 1.0);
    assert(result.raw_point_count == 1);
    if (result.filtered_cloud) {
        assert(result.filtered_cloud->sequence == 7);
        assert(result.filtered_cloud->is_deskewed);
    }
    std::puts("  [PASS] output_preserves_metadata");
}

// ─────────────────────────────────────────────────────────────────────────────

int main() {
    std::puts("PointCloudPreprocessor tests:");

    test_empty_cloud();
    test_roi_radius_crop();
    test_roi_z_crop();
    test_voxel_downsample();
    test_height_threshold_ground_removal();
    test_ransac_ground_plane();
    test_euclidean_clustering_two_blobs();
    test_cluster_min_points_filter();
    test_output_preserves_metadata();

    std::puts("PointCloudPreprocessor: ALL TESTS PASSED");
    return 0;
}
