// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Evaluation Framework: Evaluator Implementation
// ─────────────────────────────────────────────────────────────────────────────

#include "eval/evaluator.h"
#include "eval/trajectory_alignment.h"
#include "eval/trajectory_io.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>

namespace eval {

// ═════════════════════════════════════════════════════════════════════════════
//  Helpers
// ═════════════════════════════════════════════════════════════════════════════

static double vec_dist(const std::array<double,3>& a,
                       const std::array<double,3>& b) {
    double dx = a[0]-b[0], dy = a[1]-b[1], dz = a[2]-b[2];
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

/// Quaternion conjugate (= inverse for unit quaternions).  [w,x,y,z].
static std::array<double,4> quat_conj(const std::array<double,4>& q) {
    return {q[0], -q[1], -q[2], -q[3]};
}

/// Hamilton quaternion product:  r = a * b.  Convention [w,x,y,z].
static std::array<double,4> quat_mul(const std::array<double,4>& a,
                                      const std::array<double,4>& b) {
    return {
        a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
        a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
        a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
        a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]
    };
}

/// Rotate a 3D vector by a unit quaternion:  v' = q * v * q⁻¹.
static std::array<double,3> quat_rotate(const std::array<double,4>& q,
                                         const std::array<double,3>& v) {
    // Optimised form (avoids double quaternion multiply).
    //   t = 2 * (q_xyz × v)
    //   v' = v + q_w * t + (q_xyz × t)
    double tx = 2.0 * (q[2]*v[2] - q[3]*v[1]);
    double ty = 2.0 * (q[3]*v[0] - q[1]*v[2]);
    double tz = 2.0 * (q[1]*v[1] - q[2]*v[0]);
    return {
        v[0] + q[0]*tx + (q[2]*tz - q[3]*ty),
        v[1] + q[0]*ty + (q[3]*tx - q[1]*tz),
        v[2] + q[0]*tz + (q[1]*ty - q[2]*tx)
    };
}

/// Rotation angle of a unit quaternion in degrees.
static double quat_angle_deg(const std::array<double,4>& q) {
    double w = std::clamp(std::abs(q[0]), 0.0, 1.0);
    return 2.0 * std::acos(w) * (180.0 / 3.14159265358979323846);
}

/// Vector norm.
static double vec_norm(const std::array<double,3>& v) {
    return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

/// Find nearest GT pose by timestamp (binary search).
static size_t nearest_gt(const std::vector<GtPose>& gt, int64_t ts) {
    if (gt.empty()) return 0;
    auto it = std::lower_bound(gt.begin(), gt.end(), ts,
        [](const GtPose& g, int64_t t) { return g.timestamp_ns < t; });
    if (it == gt.end()) return gt.size() - 1;
    if (it == gt.begin()) return 0;
    auto prev = std::prev(it);
    return (std::abs(it->timestamp_ns - ts) < std::abs(prev->timestamp_ns - ts))
        ? static_cast<size_t>(it - gt.begin())
        : static_cast<size_t>(prev - gt.begin());
}

// ═════════════════════════════════════════════════════════════════════════════
//  Metric computation
// ═════════════════════════════════════════════════════════════════════════════

MetricSet Evaluator::compute(
    const PoseRecorder& recorder,
    const ResourceMonitor& resources,
    const std::vector<FrameTiming>& timings) const
{
    MetricSet m;
    const auto est = recorder.estimated();
    const auto& gt = recorder.groundTruth();
    m.num_poses     = est.size();
    m.frame_timings = timings;

    // ── Runtime metrics ─────────────────────────────────────────────────
    if (!timings.empty()) {
        std::vector<double> frame_ms;
        frame_ms.reserve(timings.size());
        for (const auto& t : timings) frame_ms.push_back(t.total_ms);

        std::sort(frame_ms.begin(), frame_ms.end());
        m.runtime.num_frames = frame_ms.size();
        m.runtime.avg_ms = std::accumulate(frame_ms.begin(), frame_ms.end(), 0.0)
                           / static_cast<double>(frame_ms.size());
        m.runtime.max_ms = frame_ms.back();
        m.runtime.p95_ms = frame_ms[static_cast<size_t>(frame_ms.size() * 0.95)];
    }

    // ── Resource metrics ────────────────────────────────────────────────
    m.resources.peak_rss_bytes = resources.peakRss();
    m.resources.avg_cpu_pct    = resources.avgCpu();

    // ── Trajectory metrics (require GT) ─────────────────────────────────
    if (gt.empty() || est.empty()) {
        std::cerr << "[eval] no ground truth — skipping ATE/RPE/drift\n";
        return m;
    }

    // Align estimated ↔ GT by nearest timestamp.
    struct AlignedPair {
        std::array<double,3> est_pos;
        std::array<double,4> est_quat;
        std::array<double,3> gt_pos;
        std::array<double,4> gt_quat;
    };
    std::vector<AlignedPair> aligned;
    aligned.reserve(est.size());

    for (const auto& e : est) {
        size_t gi = nearest_gt(gt, e.timestamp_ns);
        // Reject if timestamp gap > 50 ms.
        if (std::abs(gt[gi].timestamp_ns - e.timestamp_ns) > 50'000'000) continue;
        aligned.push_back({e.position, e.quaternion, gt[gi].position, gt[gi].quaternion});
    }

    if (aligned.size() < 2) {
        std::cerr << "[eval] too few aligned pairs (" << aligned.size() << ")\n";
        return m;
    }

    // ── SE(3)/Sim(3) alignment (if enabled) ─────────────────────────────
    if (cfg_.align_trajectories) {
        // Build point pairs.
        std::vector<PointPair> pp;
        pp.reserve(aligned.size());
        for (const auto& a : aligned) {
            pp.push_back({a.est_pos, a.gt_pos});
        }

        AlignmentConfig acfg;
        acfg.estimate_scale        = cfg_.estimate_scale;
        acfg.outlier_rejection     = cfg_.outlier_rejection;
        acfg.outlier_threshold_sigma = cfg_.outlier_threshold_sigma;

        auto alignment = alignTrajectories(pp, acfg);

        std::cerr << "[eval] alignment: RMSE " << alignment.rmse_before
                  << " → " << alignment.rmse_after << " m";
        if (acfg.estimate_scale)
            std::cerr << ",  scale=" << alignment.scale;
        if (acfg.outlier_rejection)
            std::cerr << ",  inliers=" << alignment.inlier_count
                      << "/" << alignment.total_pairs;
        std::cerr << "\n";

        // Apply alignment to estimated positions and orientations.
        // Both must be transformed into the GT frame so RPE does not
        // mix frames when computing relative transforms.
        for (auto& a : aligned) {
            a.est_pos  = applyAlignment(a.est_pos, alignment);
            a.est_quat = quat_mul(alignment.rotation, a.est_quat);
        }
    }

    // ── ATE (Absolute Trajectory Error) ─────────────────────────────────
    {
        std::vector<double> errors;
        errors.reserve(aligned.size());
        for (const auto& a : aligned) {
            errors.push_back(vec_dist(a.est_pos, a.gt_pos));
        }

        double sum_sq = 0.0;
        for (double e : errors) sum_sq += e * e;

        std::sort(errors.begin(), errors.end());
        m.ate.rmse_m   = std::sqrt(sum_sq / errors.size());
        m.ate.mean_m   = std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();
        m.ate.max_m    = errors.back();
        m.ate.median_m = errors[errors.size() / 2];
    }

    // ── RPE (Relative Pose Error) ───────────────────────────────────────
    {
        const double delta_m = cfg_.rpe_delta_m;
        std::vector<double> trans_errors, rot_errors;

        // Compute GT cumulative distances.
        std::vector<double> gt_cum_dist(aligned.size(), 0.0);
        for (size_t i = 1; i < aligned.size(); ++i) {
            gt_cum_dist[i] = gt_cum_dist[i-1] + vec_dist(aligned[i-1].gt_pos, aligned[i].gt_pos);
        }
        m.total_distance_m = gt_cum_dist.back();

        // Duration.
        if (!est.empty()) {
            m.sequence_duration_s = (est.back().timestamp_ns - est.front().timestamp_ns) / 1.0e9;
        }

        for (size_t i = 0; i < aligned.size(); ++i) {
            // Find j where GT distance from i → j ≈ delta_m.
            double target = gt_cum_dist[i] + delta_m;
            auto it = std::lower_bound(gt_cum_dist.begin() + static_cast<long>(i),
                                       gt_cum_dist.end(), target);
            if (it == gt_cum_dist.end()) break;
            size_t j = static_cast<size_t>(it - gt_cum_dist.begin());
            if (j >= aligned.size()) break;

            // ── SE(3) relative pose error ───────────────────────────
            // ΔT_gt  = T_gt_i⁻¹  · T_gt_j
            // ΔT_est = T_est_i⁻¹ · T_est_j
            // E      = ΔT_gt⁻¹   · ΔT_est

            // GT relative transform.
            auto qi_gt_inv = quat_conj(aligned[i].gt_quat);
            auto dq_gt     = quat_mul(qi_gt_inv, aligned[j].gt_quat);
            std::array<double,3> dp_gt_world = {
                aligned[j].gt_pos[0] - aligned[i].gt_pos[0],
                aligned[j].gt_pos[1] - aligned[i].gt_pos[1],
                aligned[j].gt_pos[2] - aligned[i].gt_pos[2]
            };
            auto dt_gt = quat_rotate(qi_gt_inv, dp_gt_world);

            // Estimated relative transform.
            auto qi_est_inv = quat_conj(aligned[i].est_quat);
            auto dq_est     = quat_mul(qi_est_inv, aligned[j].est_quat);
            std::array<double,3> dp_est_world = {
                aligned[j].est_pos[0] - aligned[i].est_pos[0],
                aligned[j].est_pos[1] - aligned[i].est_pos[1],
                aligned[j].est_pos[2] - aligned[i].est_pos[2]
            };
            auto dt_est = quat_rotate(qi_est_inv, dp_est_world);

            // Error transform E = ΔT_gt⁻¹ · ΔT_est.
            auto dq_gt_inv = quat_conj(dq_gt);
            auto q_err     = quat_mul(dq_gt_inv, dq_est);
            std::array<double,3> dt_diff = {
                dt_est[0] - dt_gt[0],
                dt_est[1] - dt_gt[1],
                dt_est[2] - dt_gt[2]
            };
            auto t_err = quat_rotate(dq_gt_inv, dt_diff);

            trans_errors.push_back(vec_norm(t_err));
            rot_errors.push_back(quat_angle_deg(q_err));
        }

        if (!trans_errors.empty()) {
            double sum_sq = 0.0;
            for (double e : trans_errors) sum_sq += e * e;
            m.rpe.trans_rmse_m = std::sqrt(sum_sq / trans_errors.size());
            m.rpe.delta_m = delta_m;
        }
        if (!rot_errors.empty()) {
            double sum_sq = 0.0;
            for (double e : rot_errors) sum_sq += e * e;
            m.rpe.rot_rmse_deg = std::sqrt(sum_sq / rot_errors.size());
        }
    }

    // ── Drift ───────────────────────────────────────────────────────────
    if (m.total_distance_m > 0.0) {
        // Final position error.
        double final_error = vec_dist(aligned.back().est_pos, aligned.back().gt_pos);
        m.drift.drift_pct      = (final_error / m.total_distance_m) * 100.0;
        m.drift.drift_per_100m = final_error / (m.total_distance_m / 100.0);
    }

    return m;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Output writers
// ═════════════════════════════════════════════════════════════════════════════

void Evaluator::writeCsv(const MetricSet& m, const std::string& path) {
    std::ofstream f(path);
    if (!f.is_open()) { std::cerr << "[eval] cannot write " << path << "\n"; return; }

    f << "metric,value,unit\n";
    f << "ate_rmse,"    << m.ate.rmse_m   << ",m\n";
    f << "ate_mean,"    << m.ate.mean_m   << ",m\n";
    f << "ate_max,"     << m.ate.max_m    << ",m\n";
    f << "ate_median,"  << m.ate.median_m << ",m\n";
    f << "rpe_trans_rmse," << m.rpe.trans_rmse_m << ",m\n";
    f << "rpe_rot_rmse,"   << m.rpe.rot_rmse_deg << ",deg\n";
    f << "rpe_delta,"      << m.rpe.delta_m      << ",m\n";
    f << "drift_pct,"      << m.drift.drift_pct  << ",%\n";
    f << "drift_per_100m," << m.drift.drift_per_100m << ",m\n";
    f << "runtime_avg,"    << m.runtime.avg_ms   << ",ms\n";
    f << "runtime_p95,"    << m.runtime.p95_ms   << ",ms\n";
    f << "runtime_max,"    << m.runtime.max_ms   << ",ms\n";
    f << "cpu_avg,"        << m.resources.avg_cpu_pct << ",%\n";
    f << "peak_rss,"       << m.resources.peak_rss_bytes << ",bytes\n";
    f << "num_poses,"      << m.num_poses         << ",count\n";
    f << "distance,"       << m.total_distance_m  << ",m\n";
    f << "duration,"       << m.sequence_duration_s << ",s\n";

    std::cerr << "[eval] wrote " << path << "\n";
}

void Evaluator::writeJson(const MetricSet& m, const std::string& path) {
    std::ofstream f(path);
    if (!f.is_open()) { std::cerr << "[eval] cannot write " << path << "\n"; return; }

    f << std::fixed << std::setprecision(6);
    f << "{\n";
    f << "  \"dataset\": \"" << m.dataset_name << "\",\n";
    f << "  \"ate\": {\n";
    f << "    \"rmse_m\": "   << m.ate.rmse_m   << ",\n";
    f << "    \"mean_m\": "   << m.ate.mean_m   << ",\n";
    f << "    \"max_m\": "    << m.ate.max_m    << ",\n";
    f << "    \"median_m\": " << m.ate.median_m << "\n";
    f << "  },\n";
    f << "  \"rpe\": {\n";
    f << "    \"trans_rmse_m\": " << m.rpe.trans_rmse_m << ",\n";
    f << "    \"rot_rmse_deg\": " << m.rpe.rot_rmse_deg << ",\n";
    f << "    \"delta_m\": "      << m.rpe.delta_m      << "\n";
    f << "  },\n";
    f << "  \"drift\": {\n";
    f << "    \"drift_pct\": "      << m.drift.drift_pct      << ",\n";
    f << "    \"drift_per_100m\": " << m.drift.drift_per_100m << "\n";
    f << "  },\n";
    f << "  \"runtime\": {\n";
    f << "    \"avg_ms\": "    << m.runtime.avg_ms   << ",\n";
    f << "    \"p95_ms\": "    << m.runtime.p95_ms   << ",\n";
    f << "    \"max_ms\": "    << m.runtime.max_ms   << ",\n";
    f << "    \"num_frames\": " << m.runtime.num_frames << "\n";
    f << "  },\n";
    f << "  \"resources\": {\n";
    f << "    \"avg_cpu_pct\": "    << m.resources.avg_cpu_pct    << ",\n";
    f << "    \"peak_rss_bytes\": " << m.resources.peak_rss_bytes << "\n";
    f << "  },\n";
    f << "  \"num_poses\": "          << m.num_poses          << ",\n";
    f << "  \"total_distance_m\": "   << m.total_distance_m   << ",\n";
    f << "  \"sequence_duration_s\": " << m.sequence_duration_s << "\n";
    f << "}\n";

    std::cerr << "[eval] wrote " << path << "\n";
}

void Evaluator::writeTumTrajectory(const std::vector<StampedPose>& est,
                                    const std::string& path) {
    tum::write(est, path);
    std::cerr << "[eval] wrote " << path << " (" << est.size() << " poses)\n";
}

void Evaluator::writeTumTrajectory(const std::vector<GtPose>& gt,
                                    const std::string& path) {
    tum::write(gt, path);
    std::cerr << "[eval] wrote " << path << " (" << gt.size() << " poses)\n";
}

} // namespace eval
