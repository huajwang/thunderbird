// ─────────────────────────────────────────────────────────────────────────────
// Test — PerceptionEngine integration (thread pipeline + ring buffers)
// ─────────────────────────────────────────────────────────────────────────────
//
// Covers:
//   • initialize / start / stop / shutdown lifecycle
//   • feedSlamOutput is non-blocking
//   • Full pipeline: SlamOutput → preprocess → detect → track → output
//   • Callback API fires on new tracked objects
//   • isRunning() state transitions
//   • Repeated start/stop cycles are safe
//
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/perception/perception_engine.h"
#include "thunderbird/perception/perception_config.h"
#include "thunderbird/perception/perception_types.h"
#include "thunderbird/odom/slam_types.h"

#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <memory>
#include <thread>
#include <vector>

using namespace thunderbird::perception;
using namespace thunderbird::odom;

// ─────────────────────────────────────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Build a synthetic SlamOutput with a point cloud containing a vehicle-
/// sized blob that the CpuClusterDetector should classify.
static std::shared_ptr<const SlamOutput>
makeSlamOutput(int64_t ts_ns, double obj_x, double obj_y) {
    auto out = std::make_shared<SlamOutput>();
    out->timestamp_ns = ts_ns;
    out->pose.timestamp_ns = ts_ns;
    out->pose.quaternion = {1, 0, 0, 0};
    out->pose.position = {0, 0, 0};

    auto cloud = std::make_shared<PointCloudFrame>();
    cloud->timestamp_ns = ts_ns;
    cloud->is_deskewed  = true;

    // Ground plane at z = 0 (many points)
    for (int i = 0; i < 20; ++i) {
        for (int j = 0; j < 20; ++j) {
            PointXYZIT p;
            p.x = static_cast<float>(i * 0.5 - 5.0);
            p.y = static_cast<float>(j * 0.5 - 5.0);
            p.z = 0.0f;
            p.intensity = 30.0f;
            cloud->points.push_back(p);
        }
    }

    // Vehicle-sized blob: 4 × 1.8 × 1.5 m above ground
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 3; ++k) {
                PointXYZIT p;
                p.x = static_cast<float>(obj_x - 2.0 + i * 0.8);
                p.y = static_cast<float>(obj_y - 0.9 + j * 0.45);
                p.z = static_cast<float>(0.5 + k * 0.5);
                p.intensity = 120.0f;
                cloud->points.push_back(p);
            }
        }
    }

    out->deskewed_cloud = std::move(cloud);
    return out;
}

/// Wait until a predicate is true, or timeout.
template<typename Pred>
static bool waitFor(Pred pred, int timeout_ms = 3000) {
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(timeout_ms);
    while (!pred()) {
        if (std::chrono::steady_clock::now() > deadline) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Tests
// ─────────────────────────────────────────────────────────────────────────────

static void test_lifecycle() {
    PerceptionEngine engine;
    assert(!engine.isRunning());

    PerceptionConfig cfg;
    cfg.detector.backend = DetectorBackend::CpuCluster;
    cfg.detector.confidence_threshold = 0.1f;
    assert(engine.initialize(cfg));

    engine.start();
    assert(engine.isRunning());

    engine.stop();
    assert(!engine.isRunning());

    engine.shutdown();
    std::puts("  [PASS] lifecycle");
}

static void test_feed_is_nonblocking() {
    PerceptionEngine engine;
    PerceptionConfig cfg;
    cfg.detector.backend = DetectorBackend::CpuCluster;
    engine.initialize(cfg);
    engine.start();

    auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < 100; ++i) {
        engine.feedSlamOutput(makeSlamOutput(
            1'000'000'000LL + i * 100'000'000LL, 5.0, 0.0));
    }
    auto elapsed = std::chrono::steady_clock::now() - start;
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();

    // 100 pushes should take < 10 ms total (non-blocking)
    assert(us < 10'000);

    engine.stop();
    engine.shutdown();
    std::puts("  [PASS] feed_is_nonblocking");
}

static void test_full_pipeline_pull() {
    PerceptionEngine engine;
    PerceptionConfig cfg;
    cfg.detector.backend = DetectorBackend::CpuCluster;
    cfg.detector.confidence_threshold = 0.1f;
    cfg.tracker.confirm_hits = 2;
    cfg.tracker.association_metric = AssociationMetric::CenterDistance;
    cfg.tracker.association_gate = 10.0;
    engine.initialize(cfg);
    engine.start();

    // Feed several frames with a stationary vehicle
    for (int i = 0; i < 10; ++i) {
        auto slam_out = makeSlamOutput(
            1'000'000'000LL + i * 100'000'000LL, 5.0, 0.0);
        engine.feedSlamOutput(slam_out);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Poll for tracked objects
    std::shared_ptr<const TrackedObjectList> result;
    bool got = waitFor([&]() { return engine.getDetectedObjects(result); });

    assert(got);
    assert(result != nullptr);
    // Pipeline should have produced at least one tracked object
    assert(result->objects.size() >= 1);

    engine.stop();
    engine.shutdown();
    std::puts("  [PASS] full_pipeline_pull");
}

static void test_callback_api() {
    PerceptionEngine engine;
    PerceptionConfig cfg;
    cfg.detector.backend = DetectorBackend::CpuCluster;
    cfg.detector.confidence_threshold = 0.1f;
    cfg.tracker.confirm_hits = 2;
    cfg.tracker.association_metric = AssociationMetric::CenterDistance;
    cfg.tracker.association_gate = 10.0;
    engine.initialize(cfg);

    std::atomic<int> callback_count{0};
    engine.onTrackedObjects([&](std::shared_ptr<const TrackedObjectList>) {
        callback_count.fetch_add(1, std::memory_order_relaxed);
    });

    engine.start();

    for (int i = 0; i < 10; ++i) {
        engine.feedSlamOutput(makeSlamOutput(
            1'000'000'000LL + i * 100'000'000LL, 5.0, 0.0));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Wait for at least one callback
    bool got = waitFor([&]() {
        return callback_count.load(std::memory_order_relaxed) > 0;
    });
    assert(got);

    engine.stop();
    engine.shutdown();
    std::puts("  [PASS] callback_api");
}

static void test_repeated_start_stop() {
    PerceptionEngine engine;
    PerceptionConfig cfg;
    cfg.detector.backend = DetectorBackend::CpuCluster;
    engine.initialize(cfg);

    for (int cycle = 0; cycle < 3; ++cycle) {
        engine.start();
        assert(engine.isRunning());

        engine.feedSlamOutput(makeSlamOutput(
            1'000'000'000LL + cycle * 1'000'000'000LL, 5.0, 0.0));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        engine.stop();
        assert(!engine.isRunning());
    }

    engine.shutdown();
    std::puts("  [PASS] repeated_start_stop");
}

// ─────────────────────────────────────────────────────────────────────────────

int main() {
    std::puts("PerceptionEngine tests:");

    test_lifecycle();
    test_feed_is_nonblocking();
    test_full_pipeline_pull();
    test_callback_api();
    test_repeated_start_stop();

    std::puts("PerceptionEngine: ALL TESTS PASSED");
    return 0;
}
