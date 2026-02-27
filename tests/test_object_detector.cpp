// ─────────────────────────────────────────────────────────────────────────────
// Test — CpuClusterDetector smoke tests
// ─────────────────────────────────────────────────────────────────────────────
//
// Verifies the CPU-based cluster detector:
//   • Factory creates a CpuClusterDetector
//   • Empty input yields no detections
//   • Person-sized cluster is classified as Person
//   • Vehicle-sized cluster is classified as Vehicle
//   • Pole-sized cluster is classified as Pole
//   • Cyclist-shaped cluster is classified as Cyclist
//   • Noise (very few points) is rejected
//   • Confidence threshold filtering works
//   • max_detections cap is respected
//
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/perception/object_detector.h"
#include "thunderbird/perception/perception_config.h"
#include "thunderbird/perception/perception_types.h"
#include "thunderbird/odom/slam_types.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

using namespace thunderbird::perception;
using namespace thunderbird::odom;

// ─────────────────────────────────────────────────────────────────────────────
//  Helpers — build synthetic point clouds with labelled clusters
// ─────────────────────────────────────────────────────────────────────────────

/// Fill a rectangular prism of points in the cloud.  All points get the
/// same cluster label.  Returns the label used.
static uint32_t addCluster(
    std::vector<PointXYZIT>& pts,
    std::vector<uint32_t>& labels,
    uint32_t cluster_id,
    double cx, double cy, double cz,
    double length, double width, double height,
    int points_per_axis = 4)
{
    const double dl = length / points_per_axis;
    const double dw = width  / points_per_axis;
    const double dh = height / points_per_axis;

    for (int i = 0; i < points_per_axis; ++i) {
        for (int j = 0; j < points_per_axis; ++j) {
            for (int k = 0; k < points_per_axis; ++k) {
                PointXYZIT p;
                p.x = static_cast<float>(cx - length / 2 + dl * (i + 0.5));
                p.y = static_cast<float>(cy - width  / 2 + dw * (j + 0.5));
                p.z = static_cast<float>(cz - height / 2 + dh * (k + 0.5));
                p.intensity = 100.0f;
                p.dt_ns = 0;
                pts.push_back(p);
                labels.push_back(cluster_id);
            }
        }
    }
    return cluster_id;
}

/// Build a DetectionInput from accumulated points / labels.
static DetectionInput makeInput(
    std::vector<PointXYZIT>& pts,
    std::vector<uint32_t>& labels,
    uint32_t num_clusters)
{
    auto cloud = std::make_shared<PointCloudFrame>();
    cloud->timestamp_ns = 1'000'000'000;
    cloud->is_deskewed = true;
    cloud->points = std::move(pts);

    DetectionInput input;
    input.timestamp_ns  = cloud->timestamp_ns;
    input.filtered_cloud = cloud;
    input.cluster_labels = std::move(labels);
    input.num_clusters   = num_clusters;
    input.raw_point_count = static_cast<uint32_t>(cloud->points.size());
    return input;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Tests
// ─────────────────────────────────────────────────────────────────────────────

static void test_factory_creates_cpu_detector() {
    PerceptionConfig cfg;
    cfg.detector.backend = DetectorBackend::CpuCluster;
    auto det = ObjectDetector::create(cfg);
    assert(det != nullptr);
    assert(!det->uses_gpu());
    assert(det->initialize(cfg));
    std::puts("  [PASS] factory_creates_cpu_detector");
}

static void test_empty_input() {
    PerceptionConfig cfg;
    auto det = ObjectDetector::create(cfg);
    det->initialize(cfg);

    DetectionInput input;  // no cloud, no clusters
    auto result = det->detect(input);
    assert(result.detections.empty());
    std::puts("  [PASS] empty_input");
}

static void test_person_cluster() {
    PerceptionConfig cfg;
    cfg.detector.confidence_threshold = 0.1f;  // low threshold to see everything
    auto det = ObjectDetector::create(cfg);
    det->initialize(cfg);

    std::vector<PointXYZIT> pts;
    std::vector<uint32_t> labels;
    // Person: ~0.6 m wide, ~0.6 m long, ~1.7 m tall (sweet spot)
    addCluster(pts, labels, 1, 5.0, 0.0, 0.85, 0.6, 0.6, 1.7, 3);

    auto input = makeInput(pts, labels, 1);
    auto result = det->detect(input);

    assert(result.detections.size() == 1);
    assert(result.detections[0].label == ObjectClass::Person);
    assert(result.detections[0].confidence >= 0.5f);
    std::puts("  [PASS] person_cluster");
}

static void test_vehicle_cluster() {
    PerceptionConfig cfg;
    cfg.detector.confidence_threshold = 0.1f;
    auto det = ObjectDetector::create(cfg);
    det->initialize(cfg);

    std::vector<PointXYZIT> pts;
    std::vector<uint32_t> labels;
    // Vehicle: ~4.0 m long, ~1.8 m wide, ~1.5 m tall
    addCluster(pts, labels, 1, 10.0, 5.0, 0.75, 4.0, 1.8, 1.5, 4);

    auto input = makeInput(pts, labels, 1);
    auto result = det->detect(input);

    assert(result.detections.size() == 1);
    assert(result.detections[0].label == ObjectClass::Vehicle);
    assert(result.detections[0].confidence >= 0.5f);
    std::puts("  [PASS] vehicle_cluster");
}

static void test_pole_cluster() {
    PerceptionConfig cfg;
    cfg.detector.confidence_threshold = 0.1f;
    auto det = ObjectDetector::create(cfg);
    det->initialize(cfg);

    std::vector<PointXYZIT> pts;
    std::vector<uint32_t> labels;
    // Pole: very tall (5 m) and thin (0.3 × 0.3)
    // Use enough points_per_axis so the z-extent exceeds the Person
    // height ceiling (2.2 m) — otherwise the Person rule fires first.
    addCluster(pts, labels, 1, 3.0, -2.0, 2.5, 0.3, 0.3, 5.0);

    auto input = makeInput(pts, labels, 1);
    auto result = det->detect(input);

    assert(result.detections.size() == 1);
    assert(result.detections[0].label == ObjectClass::Pole);
    assert(result.detections[0].confidence >= 0.5f);
    std::puts("  [PASS] pole_cluster");
}

static void test_cyclist_cluster() {
    PerceptionConfig cfg;
    cfg.detector.confidence_threshold = 0.1f;
    auto det = ObjectDetector::create(cfg);
    det->initialize(cfg);

    std::vector<PointXYZIT> pts;
    std::vector<uint32_t> labels;
    // Cyclist: ~1.8 m long × 0.6 m wide × 1.7 m tall, elongated
    addCluster(pts, labels, 1, 8.0, 0.0, 0.85, 1.8, 0.6, 1.7, 3);

    auto input = makeInput(pts, labels, 1);
    auto result = det->detect(input);

    assert(result.detections.size() == 1);
    assert(result.detections[0].label == ObjectClass::Cyclist);
    assert(result.detections[0].confidence >= 0.3f);
    std::puts("  [PASS] cyclist_cluster");
}

static void test_noise_rejected() {
    PerceptionConfig cfg;
    cfg.detector.confidence_threshold = 0.1f;
    auto det = ObjectDetector::create(cfg);
    det->initialize(cfg);

    std::vector<PointXYZIT> pts;
    std::vector<uint32_t> labels;
    // Only 2 points — should be classified as noise (confidence 0)
    for (int i = 0; i < 2; ++i) {
        PointXYZIT p;
        p.x = static_cast<float>(1.0 + i * 0.1);
        p.y = 0;
        p.z = 0;
        pts.push_back(p);
        labels.push_back(1);
    }

    auto input = makeInput(pts, labels, 1);
    auto result = det->detect(input);

    // confidence 0.0 < threshold 0.1 → filtered out
    assert(result.detections.empty());
    std::puts("  [PASS] noise_rejected");
}

static void test_confidence_threshold_filtering() {
    PerceptionConfig cfg;
    cfg.detector.confidence_threshold = 0.60f;  // above person-base (0.55)
    auto det = ObjectDetector::create(cfg);
    det->initialize(cfg);

    std::vector<PointXYZIT> pts;
    std::vector<uint32_t> labels;

    // Person (base confidence 0.55 → below 0.60 threshold → filtered)
    // Put height outside sweet spot so we get base confidence only
    addCluster(pts, labels, 1, 5.0, 0.0, 1.05, 0.5, 0.5, 2.1, 3);

    // Vehicle with good confidence (0.70 → above threshold → kept)
    addCluster(pts, labels, 2, 15.0, 5.0, 0.75, 4.0, 1.8, 1.5, 4);

    auto input = makeInput(pts, labels, 2);
    auto result = det->detect(input);

    assert(result.detections.size() == 1);
    assert(result.detections[0].label == ObjectClass::Vehicle);
    std::puts("  [PASS] confidence_threshold_filtering");
}

static void test_max_detections_cap() {
    PerceptionConfig cfg;
    cfg.detector.confidence_threshold = 0.1f;
    cfg.detector.max_detections = 2;
    auto det = ObjectDetector::create(cfg);
    det->initialize(cfg);

    std::vector<PointXYZIT> pts;
    std::vector<uint32_t> labels;

    // Place 4 person-sized clusters
    for (int i = 0; i < 4; ++i) {
        addCluster(pts, labels, static_cast<uint32_t>(i + 1),
                   5.0 * (i + 1), 0.0, 0.85, 0.6, 0.6, 1.7, 3);
    }

    auto input = makeInput(pts, labels, 4);
    auto result = det->detect(input);

    assert(result.detections.size() <= 2);
    std::puts("  [PASS] max_detections_cap");
}

static void test_multiple_clusters() {
    PerceptionConfig cfg;
    cfg.detector.confidence_threshold = 0.1f;
    auto det = ObjectDetector::create(cfg);
    det->initialize(cfg);

    std::vector<PointXYZIT> pts;
    std::vector<uint32_t> labels;

    // Person + vehicle at different locations
    addCluster(pts, labels, 1, 5.0, 0.0, 0.85, 0.6, 0.6, 1.7, 3);
    addCluster(pts, labels, 2, 20.0, 10.0, 0.75, 4.0, 1.8, 1.5, 4);

    auto input = makeInput(pts, labels, 2);
    auto result = det->detect(input);

    assert(result.detections.size() == 2);

    // Check both expected classes present (order: cluster 1 first)
    bool found_person = false, found_vehicle = false;
    for (auto& d : result.detections) {
        if (d.label == ObjectClass::Person)  found_person = true;
        if (d.label == ObjectClass::Vehicle) found_vehicle = true;
    }
    assert(found_person);
    assert(found_vehicle);
    std::puts("  [PASS] multiple_clusters");
}

static void test_teardown_reinitialize() {
    PerceptionConfig cfg;
    cfg.detector.confidence_threshold = 0.1f;
    auto det = ObjectDetector::create(cfg);
    det->initialize(cfg);

    // Detect once
    std::vector<PointXYZIT> pts;
    std::vector<uint32_t> labels;
    addCluster(pts, labels, 1, 5.0, 0.0, 0.85, 0.6, 0.6, 1.7, 3);
    auto input = makeInput(pts, labels, 1);
    auto result1 = det->detect(input);
    assert(!result1.detections.empty());

    // Teardown + reinitialize
    det->teardown();
    assert(det->initialize(cfg));

    // Detect again
    std::vector<PointXYZIT> pts2;
    std::vector<uint32_t> labels2;
    addCluster(pts2, labels2, 1, 5.0, 0.0, 0.85, 0.6, 0.6, 1.7, 3);
    input = makeInput(pts2, labels2, 1);
    auto result2 = det->detect(input);
    assert(!result2.detections.empty());

    std::puts("  [PASS] teardown_reinitialize");
}

// ─────────────────────────────────────────────────────────────────────────────

int main() {
    std::puts("CpuClusterDetector tests:");

    test_factory_creates_cpu_detector();
    test_empty_input();
    test_person_cluster();
    test_vehicle_cluster();
    test_pole_cluster();
    test_cyclist_cluster();
    test_noise_rejected();
    test_confidence_threshold_filtering();
    test_max_detections_cap();
    test_multiple_clusters();
    test_teardown_reinitialize();

    std::puts("CpuClusterDetector: ALL TESTS PASSED");
    return 0;
}
