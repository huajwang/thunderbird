// ─────────────────────────────────────────────────────────────────────────────
// Test — MultiObjectTracker smoke tests
// ─────────────────────────────────────────────────────────────────────────────
//
// Verifies the 3D multi-object tracker:
//   • Construction with default config
//   • Empty frame produces no tracks
//   • Single detection spawns a tentative track
//   • Repeated detections promote track to Confirmed
//   • Missed detections eventually delete track
//   • Multiple objects tracked independently
//   • reset() clears all state
//   • Track IDs are monotonically increasing and never reused
//   • Velocity estimation after several frames
//
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/perception/multi_object_tracker.h"
#include "thunderbird/perception/perception_config.h"
#include "thunderbird/perception/perception_types.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <set>

using namespace thunderbird::perception;

// ─────────────────────────────────────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Build a DetectionFrame with one detection at the given position/class.
static DetectionFrame makeFrame(int64_t ts_ns,
                                double x, double y, double z,
                                ObjectClass label = ObjectClass::Vehicle,
                                float conf = 0.8f)
{
    DetectionFrame f;
    f.timestamp_ns = ts_ns;

    Detection3D d;
    d.bbox.center[0] = x;
    d.bbox.center[1] = y;
    d.bbox.center[2] = z;
    d.bbox.extent[0] = 4.0;
    d.bbox.extent[1] = 1.8;
    d.bbox.extent[2] = 1.5;
    d.bbox.yaw       = 0.0;
    d.label          = label;
    d.confidence     = conf;
    d.cluster_id     = 1;
    d.num_points     = 100;

    f.detections.push_back(d);
    return f;
}

/// Build a DetectionFrame with two detections at given positions.
static DetectionFrame makeTwoDetFrame(int64_t ts_ns,
                                      double x1, double y1,
                                      double x2, double y2)
{
    DetectionFrame f;
    f.timestamp_ns = ts_ns;

    auto addDet = [&](double x, double y, uint32_t cid) {
        Detection3D d;
        d.bbox.center[0] = x;
        d.bbox.center[1] = y;
        d.bbox.center[2] = 0.75;
        d.bbox.extent[0] = 4.0;
        d.bbox.extent[1] = 1.8;
        d.bbox.extent[2] = 1.5;
        d.label      = ObjectClass::Vehicle;
        d.confidence = 0.8f;
        d.cluster_id = cid;
        d.num_points = 100;
        f.detections.push_back(d);
    };

    addDet(x1, y1, 1);
    addDet(x2, y2, 2);
    return f;
}

/// Build an empty DetectionFrame at the given timestamp.
static DetectionFrame makeEmptyFrame(int64_t ts_ns) {
    DetectionFrame f;
    f.timestamp_ns = ts_ns;
    return f;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Tests
// ─────────────────────────────────────────────────────────────────────────────

static void test_construction() {
    TrackerConfig cfg;
    MultiObjectTracker tracker(cfg);
    assert(tracker.activeTrackCount() == 0);
    std::puts("  [PASS] construction");
}

static void test_empty_frame() {
    TrackerConfig cfg;
    MultiObjectTracker tracker(cfg);

    auto result = tracker.update(makeEmptyFrame(1'000'000'000));
    assert(result.objects.empty());
    std::puts("  [PASS] empty_frame");
}

static void test_single_detection_spawns_track() {
    TrackerConfig cfg;
    cfg.association_metric = AssociationMetric::CenterDistance;
    MultiObjectTracker tracker(cfg);

    auto frame = makeFrame(1'000'000'000, 10.0, 5.0, 0.75);
    auto result = tracker.update(frame);

    // After first detection, should have at least 1 active track internally
    assert(tracker.activeTrackCount() >= 1);
    std::puts("  [PASS] single_detection_spawns_track");
}

static void test_confirm_after_hits() {
    TrackerConfig cfg;
    cfg.confirm_hits    = 3;
    cfg.max_coast_frames = 5;
    cfg.association_metric = AssociationMetric::CenterDistance;
    cfg.association_gate = 10.0;
    MultiObjectTracker tracker(cfg);

    TrackedObjectList result;
    // Feed the same detection for confirm_hits frames
    for (int i = 0; i < cfg.confirm_hits + 1; ++i) {
        int64_t ts = 1'000'000'000LL + i * 100'000'000LL;  // 10 Hz
        result = tracker.update(makeFrame(ts, 10.0, 5.0, 0.75));
    }

    // Should have a confirmed track in the output
    bool has_confirmed = false;
    for (auto& obj : result.objects) {
        if (obj.state == TrackState::Confirmed) {
            has_confirmed = true;
            break;
        }
    }
    assert(has_confirmed);
    std::puts("  [PASS] confirm_after_hits");
}

static void test_coasting_on_miss() {
    TrackerConfig cfg;
    cfg.confirm_hits    = 2;
    cfg.max_coast_frames = 3;
    cfg.association_metric = AssociationMetric::CenterDistance;
    cfg.association_gate = 10.0;
    MultiObjectTracker tracker(cfg);

    // Confirm a track (feed enough hits)
    for (int i = 0; i < cfg.confirm_hits + 1; ++i) {
        int64_t ts = 1'000'000'000LL + i * 100'000'000LL;
        tracker.update(makeFrame(ts, 10.0, 5.0, 0.75));
    }

    // Now miss a frame (empty)
    int64_t ts = 1'000'000'000LL + (cfg.confirm_hits + 1) * 100'000'000LL;
    auto result = tracker.update(makeEmptyFrame(ts));

    // Track should still exist (coasting) with some misses
    bool has_coasting = false;
    for (auto& obj : result.objects) {
        if (obj.state == TrackState::Coasting) {
            has_coasting = true;
        }
    }
    assert(has_coasting);
    std::puts("  [PASS] coasting_on_miss");
}

static void test_track_deletion_after_max_coast() {
    TrackerConfig cfg;
    cfg.confirm_hits     = 2;
    cfg.max_coast_frames = 3;
    cfg.association_metric = AssociationMetric::CenterDistance;
    cfg.association_gate = 10.0;
    MultiObjectTracker tracker(cfg);

    // Confirm a track
    for (int i = 0; i < cfg.confirm_hits + 1; ++i) {
        int64_t ts = 1'000'000'000LL + i * 100'000'000LL;
        tracker.update(makeFrame(ts, 10.0, 5.0, 0.75));
    }

    // Miss for max_coast_frames + margin
    for (int i = 0; i < cfg.max_coast_frames + 2; ++i) {
        int64_t ts = 1'000'000'000LL +
                     (cfg.confirm_hits + 1 + i) * 100'000'000LL;
        tracker.update(makeEmptyFrame(ts));
    }

    // Track should be deleted
    assert(tracker.activeTrackCount() == 0);
    std::puts("  [PASS] track_deletion_after_max_coast");
}

static void test_multiple_objects() {
    TrackerConfig cfg;
    cfg.confirm_hits    = 2;
    cfg.max_coast_frames = 5;
    cfg.association_metric = AssociationMetric::CenterDistance;
    cfg.association_gate = 10.0;
    MultiObjectTracker tracker(cfg);

    TrackedObjectList result;
    // Feed two objects for enough frames to confirm
    for (int i = 0; i < cfg.confirm_hits + 1; ++i) {
        int64_t ts = 1'000'000'000LL + i * 100'000'000LL;
        result = tracker.update(makeTwoDetFrame(
            ts,
            10.0, 5.0,   // object 1 — stationary
            30.0, -5.0   // object 2 — far away
        ));
    }

    // Should have at least 2 confirmed tracks
    int confirmed = 0;
    for (auto& obj : result.objects) {
        if (obj.state == TrackState::Confirmed) ++confirmed;
    }
    assert(confirmed >= 2);
    std::puts("  [PASS] multiple_objects");
}

static void test_reset() {
    TrackerConfig cfg;
    cfg.association_metric = AssociationMetric::CenterDistance;
    cfg.association_gate = 10.0;
    MultiObjectTracker tracker(cfg);

    // Create some tracks
    for (int i = 0; i < 5; ++i) {
        int64_t ts = 1'000'000'000LL + i * 100'000'000LL;
        tracker.update(makeFrame(ts, 10.0, 5.0, 0.75));
    }
    assert(tracker.activeTrackCount() > 0);

    tracker.reset();
    assert(tracker.activeTrackCount() == 0);
    std::puts("  [PASS] reset");
}

static void test_track_ids_monotonic() {
    TrackerConfig cfg;
    cfg.confirm_hits    = 1;
    cfg.max_coast_frames = 1;
    cfg.tentative_max_misses = 1;
    cfg.association_metric = AssociationMetric::CenterDistance;
    cfg.association_gate = 10.0;
    MultiObjectTracker tracker(cfg);

    std::set<uint64_t> all_ids;
    uint64_t max_new_id = 0;

    // Feed detections, creating new tracks.  Tracks from earlier frames
    // may reappear (coasting), so we only check that *new* IDs are
    // monotonically increasing and that IDs are unique within a frame.
    for (int i = 0; i < 5; ++i) {
        int64_t ts = 1'000'000'000LL + i * 100'000'000LL;
        // Move detection far each time to force new track creation
        auto result = tracker.update(
            makeFrame(ts, 10.0 + i * 50.0, 0.0, 0.75));

        std::set<uint64_t> frame_ids;
        for (auto& obj : result.objects) {
            // No duplicate IDs within a single frame
            assert(frame_ids.find(obj.track_id) == frame_ids.end());
            frame_ids.insert(obj.track_id);

            // First time we see this ID, it must be larger than all
            // previously seen IDs (monotonic assignment).
            if (all_ids.find(obj.track_id) == all_ids.end()) {
                assert(obj.track_id > max_new_id);
                max_new_id = obj.track_id;
            }
            all_ids.insert(obj.track_id);
        }
    }

    // We should have created at least 5 distinct tracks
    assert(all_ids.size() >= 5);

    std::puts("  [PASS] track_ids_monotonic");
}

static void test_velocity_estimation() {
    TrackerConfig cfg;
    cfg.confirm_hits    = 2;
    cfg.max_coast_frames = 5;
    cfg.association_metric = AssociationMetric::CenterDistance;
    cfg.association_gate = 15.0;
    MultiObjectTracker tracker(cfg);

    // Object moving at ~5 m/s in X
    const double vx = 5.0;
    const double dt = 0.1;  // 10 Hz
    TrackedObjectList result;

    for (int i = 0; i < 10; ++i) {
        int64_t ts = 1'000'000'000LL + static_cast<int64_t>(i * dt * 1e9);
        double x = 10.0 + vx * i * dt;
        result = tracker.update(makeFrame(ts, x, 0.0, 0.75));
    }

    // After 10 frames the KF should have a reasonable velocity estimate
    assert(!result.objects.empty());
    bool found_good_vel = false;
    for (auto& obj : result.objects) {
        if (obj.state == TrackState::Confirmed) {
            // Velocity should be in the right ballpark (within 50%)
            if (std::abs(obj.velocity[0] - vx) < vx * 0.5) {
                found_good_vel = true;
            }
        }
    }
    assert(found_good_vel);
    std::puts("  [PASS] velocity_estimation");
}

static void test_move_constructor() {
    TrackerConfig cfg;
    MultiObjectTracker t1(cfg);
    t1.update(makeFrame(1'000'000'000, 10.0, 5.0, 0.75));

    MultiObjectTracker t2(std::move(t1));
    assert(t2.activeTrackCount() >= 1);
    std::puts("  [PASS] move_constructor");
}

// ─────────────────────────────────────────────────────────────────────────────

int main() {
    std::puts("MultiObjectTracker tests:");

    test_construction();
    test_empty_frame();
    test_single_detection_spawns_track();
    test_confirm_after_hits();
    test_coasting_on_miss();
    test_track_deletion_after_max_coast();
    test_multiple_objects();
    test_reset();
    test_track_ids_monotonic();
    test_velocity_estimation();
    test_move_constructor();

    std::puts("MultiObjectTracker: ALL TESTS PASSED");
    return 0;
}
