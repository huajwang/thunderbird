// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — 3D Multi-Object Tracker implementation
// ─────────────────────────────────────────────────────────────────────────────
//
// Full tracking pipeline per frame:
//
//   1. Predict   — advance each track's KF by dt seconds.
//   2. Gate      — build cost matrix; reject implausible pairs.
//   3. Associate — Hungarian algorithm for global optimum assignment.
//   4. Update    — KF measurement update for matched tracks.
//   5. Coast     — increment miss counter for unmatched tracks.
//   6. Birth     — spawn tentative tracks for unmatched detections.
//   7. Manage    — promote / demote / delete tracks per lifecycle rules.
//   8. Export    — copy Confirmed + Coasting tracks to output list.
//
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/perception/multi_object_tracker.h"

#include "hungarian.h"
#include "kalman_filter_cv.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <vector>

namespace thunderbird::perception {

// ═════════════════════════════════════════════════════════════════════════════
//  Internal track representation
// ═════════════════════════════════════════════════════════════════════════════

namespace {

struct InternalTrack {
    uint64_t    track_id{0};
    TrackState  state{TrackState::Tentative};
    ObjectClass label{ObjectClass::Unknown};
    float       confidence{0.0f};

    // Bounding box extent (length, width, height) — filtered with EMA.
    double extent[3]{};

    // Kalman filter for [px, py, pz, yaw, vx, vy, vz].
    detail::KalmanFilterCV kf;

    int  age_frames{0};
    int  hits{0};
    int  consecutive_misses{0};

    // Yaw-rate estimate (simple finite difference).
    double prev_yaw{0.0};
    double yaw_rate{0.0};
};

/// Portable π constant (avoids non-standard M_PI).
static constexpr double kPi = 3.14159265358979323846;

/// Normalise angle to [-π, π].
inline double normalise_angle(double a) {
    while (a >  kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}

/// Compute 3D IoU between two axis-aligned bounding boxes (approximate —
/// ignores yaw rotation for speed; sufficient for gating / cost).
inline double iou_3d_approx(const BBox3D& a, const BBox3D& b) {
    // Half-extents.
    const double ahx = a.extent[0] * 0.5, ahy = a.extent[1] * 0.5, ahz = a.extent[2] * 0.5;
    const double bhx = b.extent[0] * 0.5, bhy = b.extent[1] * 0.5, bhz = b.extent[2] * 0.5;

    // Overlap per axis.
    auto overlap = [](double ca, double ha, double cb, double hb) -> double {
        const double lo = std::max(ca - ha, cb - hb);
        const double hi = std::min(ca + ha, cb + hb);
        return std::max(0.0, hi - lo);
    };

    const double ox = overlap(a.center[0], ahx, b.center[0], bhx);
    const double oy = overlap(a.center[1], ahy, b.center[1], bhy);
    const double oz = overlap(a.center[2], ahz, b.center[2], bhz);

    const double inter = ox * oy * oz;
    if (inter <= 0.0) return 0.0;

    const double vol_a = a.extent[0] * a.extent[1] * a.extent[2];
    const double vol_b = b.extent[0] * b.extent[1] * b.extent[2];
    const double union_vol = vol_a + vol_b - inter;

    return (union_vol > 1e-9) ? (inter / union_vol) : 0.0;
}

/// Centre-distance between two BBox3D.
inline double centre_distance(const BBox3D& a, const BBox3D& b) {
    const double dx = a.center[0] - b.center[0];
    const double dy = a.center[1] - b.center[1];
    const double dz = a.center[2] - b.center[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

} // anonymous namespace

// ═════════════════════════════════════════════════════════════════════════════
//  PImpl
// ═════════════════════════════════════════════════════════════════════════════

struct MultiObjectTracker::Impl {
    TrackerConfig config;

    uint64_t next_track_id{1};
    uint32_t frame_sequence{0};
    int64_t  last_timestamp_ns{0};

    std::vector<InternalTrack> tracks;

    // Reusable buffers (avoid per-frame allocations).
    std::vector<double>              cost_matrix;
    std::vector<detail::Assignment>  assignments;
    std::vector<size_t>              unmatched_dets;
    std::vector<size_t>              unmatched_tracks;

    // ─────────────────────────────────────────────────────────────────────
    //  Build cost matrix and gate
    // ─────────────────────────────────────────────────────────────────────

    void buildCostMatrix(const std::vector<Detection3D>& detections) {
        const size_t nd = detections.size();
        const size_t nt = tracks.size();
        cost_matrix.resize(nd * nt);

        const double gate = config.association_gate;

        for (size_t di = 0; di < nd; ++di) {
            const auto& det = detections[di];

            // Predicted bbox from track state.
            for (size_t ti = 0; ti < nt; ++ti) {
                const auto& trk = tracks[ti];
                double cost = gate + 1.0;  // default: gated out

                BBox3D predicted_box;
                predicted_box.center[0] = trk.kf.px();
                predicted_box.center[1] = trk.kf.py();
                predicted_box.center[2] = trk.kf.pz();
                predicted_box.extent[0] = trk.extent[0];
                predicted_box.extent[1] = trk.extent[1];
                predicted_box.extent[2] = trk.extent[2];
                predicted_box.yaw       = trk.kf.yaw();

                switch (config.association_metric) {
                    case AssociationMetric::IoU3D: {
                        const double iou = iou_3d_approx(det.bbox, predicted_box);
                        // IoU → cost: higher IoU = lower cost.
                        // Gate: reject if IoU < (1 - gate) is too complex;
                        // instead, gate on centre distance as pre-filter.
                        const double cd = centre_distance(det.bbox, predicted_box);
                        if (cd <= gate) {
                            cost = 1.0 - iou;  // cost in [0, 1]
                        }
                        break;
                    }
                    case AssociationMetric::CenterDistance: {
                        const double cd = centre_distance(det.bbox, predicted_box);
                        if (cd <= gate) {
                            cost = cd;
                        }
                        break;
                    }
                    case AssociationMetric::Mahalanobis: {
                        const std::array<double, 4> meas = {{
                            det.bbox.center[0], det.bbox.center[1],
                            det.bbox.center[2], det.bbox.yaw
                        }};
                        const double md = trk.kf.mahalanobis(
                            meas,
                            config.measurement_noise,
                            config.process_noise_yaw);

                        if (md <= gate) {
                            cost = md;
                        }
                        break;
                    }
                }

                cost_matrix[di * nt + ti] = cost;
            }
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    //  Full tracking cycle
    // ─────────────────────────────────────────────────────────────────────

    TrackedObjectList runCycle(const DetectionFrame& detections) {
        const int64_t ts = detections.timestamp_ns;

        // Compute dt from previous frame (clamp for safety).
        double dt = 0.1;  // default 10 Hz
        if (last_timestamp_ns > 0 && ts > last_timestamp_ns) {
            dt = static_cast<double>(ts - last_timestamp_ns) * 1e-9;
            if (dt > 2.0) dt = 2.0;    // clamp: no more than 2 s
            if (dt < 0.001) dt = 0.001; // clamp: at least 1 ms
        }
        last_timestamp_ns = ts;

        // ── 1. Predict ──────────────────────────────────────────────────
        for (auto& trk : tracks) {
            trk.kf.predict(dt,
                           config.process_noise_pos,
                           config.process_noise_vel,
                           config.process_noise_yaw);
            trk.age_frames++;
        }

        const auto& dets = detections.detections;
        const size_t nd = dets.size();
        const size_t nt = tracks.size();

        // ── 2–3. Gate + Associate ───────────────────────────────────────
        if (nd > 0 && nt > 0) {
            buildCostMatrix(dets);
            detail::hungarian_solve(
                cost_matrix, nd, nt,
                config.association_gate,
                assignments, unmatched_dets, unmatched_tracks);
        } else {
            assignments.clear();
            unmatched_dets.clear();
            unmatched_tracks.clear();
            for (size_t i = 0; i < nd; ++i) unmatched_dets.push_back(i);
            for (size_t i = 0; i < nt; ++i) unmatched_tracks.push_back(i);
        }

        // ── 4. Update matched tracks ────────────────────────────────────
        for (const auto& a : assignments) {
            auto& trk = tracks[a.col];
            const auto& det = dets[a.row];

            // KF measurement update.
            const std::array<double, 4> meas = {{
                det.bbox.center[0], det.bbox.center[1],
                det.bbox.center[2], det.bbox.yaw
            }};
            trk.kf.update(meas,
                          config.measurement_noise,
                          config.process_noise_yaw);

            // Update extent with EMA smoothing.
            constexpr double alpha_ext = 0.4;
            for (int i = 0; i < 3; ++i) {
                trk.extent[i] = alpha_ext * det.bbox.extent[i] +
                                (1.0 - alpha_ext) * trk.extent[i];
            }

            // Yaw-rate via finite difference on the KF yaw.
            const double new_yaw = trk.kf.yaw();
            double dyaw = normalise_angle(new_yaw - trk.prev_yaw);
            trk.yaw_rate = (dt > 1e-6) ? (dyaw / dt) : 0.0;
            trk.prev_yaw = new_yaw;

            // Label: keep highest-confidence class.
            if (det.confidence > trk.confidence) {
                trk.label = det.label;
            }

            // Smooth confidence.
            trk.confidence = 0.6f * trk.confidence + 0.4f * det.confidence;

            trk.hits++;
            trk.consecutive_misses = 0;

            // State promotion.
            if (trk.state == TrackState::Coasting) {
                trk.state = TrackState::Confirmed;
            }
        }

        // ── 5. Coast unmatched tracks ───────────────────────────────────
        for (size_t ti : unmatched_tracks) {
            auto& trk = tracks[ti];
            trk.consecutive_misses++;

            if (trk.state == TrackState::Confirmed) {
                trk.state = TrackState::Coasting;
            }

            // Decay confidence while coasting.
            trk.confidence *= 0.9f;
        }

        // ── 6. Birth new tracks ─────────────────────────────────────────
        for (size_t di : unmatched_dets) {
            const auto& det = dets[di];

            // Reject very low-confidence detections as track seeds.
            if (det.confidence < 0.1f) continue;

            InternalTrack new_track;
            new_track.track_id = next_track_id++;
            new_track.state    = TrackState::Tentative;
            new_track.label    = det.label;
            new_track.confidence = det.confidence;

            new_track.extent[0] = det.bbox.extent[0];
            new_track.extent[1] = det.bbox.extent[1];
            new_track.extent[2] = det.bbox.extent[2];

            new_track.kf.init(
                det.bbox.center[0], det.bbox.center[1],
                det.bbox.center[2], det.bbox.yaw,
                config.measurement_noise,
                config.process_noise_vel,
                config.process_noise_yaw);

            new_track.prev_yaw = det.bbox.yaw;
            new_track.yaw_rate = 0.0;
            new_track.age_frames = 1;
            new_track.hits       = 1;
            new_track.consecutive_misses = 0;

            tracks.push_back(std::move(new_track));
        }

        // ── 7. Manage lifecycle ─────────────────────────────────────────
        // Promote: Tentative → Confirmed when hits ≥ threshold.
        for (auto& trk : tracks) {
            if (trk.state == TrackState::Tentative &&
                trk.hits >= config.confirm_hits) {
                trk.state = TrackState::Confirmed;
            }
        }

        // Delete: remove tracks that have exceeded their miss budget.
        tracks.erase(
            std::remove_if(tracks.begin(), tracks.end(),
                [this](const InternalTrack& t) {
                    if (t.state == TrackState::Tentative &&
                        t.consecutive_misses > config.tentative_max_misses)
                        return true;
                    if ((t.state == TrackState::Confirmed ||
                         t.state == TrackState::Coasting) &&
                        t.consecutive_misses > config.max_coast_frames)
                        return true;
                    return false;
                }),
            tracks.end());

        // ── 8. Export ───────────────────────────────────────────────────
        TrackedObjectList output;
        output.timestamp_ns   = ts;
        output.ego_pose       = detections.ego_pose;
        output.frame_sequence = frame_sequence++;
        output.num_detections = static_cast<uint32_t>(nd);

        for (const auto& trk : tracks) {
            // Only export confirmed + coasting tracks.
            if (trk.state != TrackState::Confirmed &&
                trk.state != TrackState::Coasting) {
                continue;
            }

            TrackedObject obj;
            obj.track_id         = trk.track_id;
            obj.state            = trk.state;
            obj.label            = trk.label;
            obj.confidence       = trk.confidence;

            obj.bbox.center[0]   = trk.kf.px();
            obj.bbox.center[1]   = trk.kf.py();
            obj.bbox.center[2]   = trk.kf.pz();
            obj.bbox.extent[0]   = trk.extent[0];
            obj.bbox.extent[1]   = trk.extent[1];
            obj.bbox.extent[2]   = trk.extent[2];
            obj.bbox.yaw         = trk.kf.yaw();

            obj.velocity[0]      = trk.kf.vx();
            obj.velocity[1]      = trk.kf.vy();
            obj.velocity[2]      = trk.kf.vz();
            obj.yaw_rate         = trk.yaw_rate;

            obj.age_frames       = trk.age_frames;
            obj.hits             = trk.hits;
            obj.consecutive_misses = trk.consecutive_misses;

            trk.kf.copyCovariance(obj.covariance);

            output.objects.push_back(obj);
        }

        return output;
    }
};

// ═════════════════════════════════════════════════════════════════════════════
//  Public API
// ═════════════════════════════════════════════════════════════════════════════

MultiObjectTracker::MultiObjectTracker(const TrackerConfig& config)
    : impl_(std::make_unique<Impl>())
{
    impl_->config = config;
}

MultiObjectTracker::~MultiObjectTracker() = default;

MultiObjectTracker::MultiObjectTracker(MultiObjectTracker&&) noexcept = default;
MultiObjectTracker& MultiObjectTracker::operator=(MultiObjectTracker&&) noexcept = default;

TrackedObjectList MultiObjectTracker::update(const DetectionFrame& detections) {
    return impl_->runCycle(detections);
}

void MultiObjectTracker::reset() {
    impl_->tracks.clear();
    impl_->next_track_id   = 1;
    impl_->frame_sequence  = 0;
    impl_->last_timestamp_ns = 0;
}

uint32_t MultiObjectTracker::activeTrackCount() const noexcept {
    return static_cast<uint32_t>(impl_->tracks.size());
}

} // namespace thunderbird::perception
