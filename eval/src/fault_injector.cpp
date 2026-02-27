// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Evaluation Framework: Fault Injection Implementation
// ─────────────────────────────────────────────────────────────────────────────

#include "eval/fault_injector.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>

namespace eval {

// ═════════════════════════════════════════════════════════════════════════════
//  FaultConfig helpers
// ═════════════════════════════════════════════════════════════════════════════

std::string FaultConfig::label() const {
    std::ostringstream ss;
    bool first = true;

    auto sep = [&]() { if (!first) ss << "+"; first = false; };

    if (lidar_drop_rate > 0.0) {
        sep();
        ss << "drop" << static_cast<int>(lidar_drop_rate * 100) << "pct";
    }
    if (imu_noise_scale > 0.0) {
        sep();
        ss << "imu_noise_x" << imu_noise_scale;
    }
    if (timestamp_jitter_ns > 0) {
        sep();
        double jitter_ms = timestamp_jitter_ns / 1.0e6;
        ss << "jitter_" << jitter_ms << "ms";
    }
    if (lidar_rate_divisor > 1) {
        sep();
        ss << "rate_div" << lidar_rate_divisor;
    }

    if (first) return "no_fault";
    return ss.str();
}

bool FaultConfig::active() const {
    return lidar_drop_rate > 0.0 ||
           imu_noise_scale > 0.0 ||
           timestamp_jitter_ns > 0 ||
           lidar_rate_divisor > 1;
}

// ═════════════════════════════════════════════════════════════════════════════
//  FaultInjectorAdapter
// ═════════════════════════════════════════════════════════════════════════════

FaultInjectorAdapter::FaultInjectorAdapter(
    std::unique_ptr<DatasetAdapter> inner,
    FaultConfig fault)
    : inner_(std::move(inner))
    , fault_(fault)
    , rng_(fault.seed)
{
    // Configure jitter distribution if needed.
    if (fault_.timestamp_jitter_ns > 0) {
        jitter_dist_ = std::uniform_int_distribution<int64_t>(
            -fault_.timestamp_jitter_ns, fault_.timestamp_jitter_ns);
    }
}

bool FaultInjectorAdapter::open(const std::string& path, size_t max_frames) {
    // Inner adapter is already opened before wrapping (typical pattern),
    // but support re-opening if needed.
    return inner_->open(path, max_frames);
}

DatasetInfo FaultInjectorAdapter::info() const {
    auto di = inner_->info();
    di.name += " [" + fault_.label() + "]";
    return di;
}

std::optional<StreamEvent> FaultInjectorAdapter::next() {
    while (true) {
        auto ev = inner_->next();
        if (!ev) return std::nullopt;  // EOF

        if (std::holds_alternative<ImuSample>(ev->payload)) {
            // ── IMU event ───────────────────────────────────────────────
            stats_.imu_total++;

            if (fault_.imu_noise_scale > 0.0) {
                auto& imu = std::get<ImuSample>(ev->payload);
                injectImuNoise(imu);
            }

            if (fault_.timestamp_jitter_ns > 0) {
                ev->timestamp_ns = jitterTimestamp(ev->timestamp_ns);
                auto& imu = std::get<ImuSample>(ev->payload);
                imu.timestamp_ns = ev->timestamp_ns;
                stats_.events_jittered++;
            }

            return ev;

        } else {
            // ── LiDAR event ─────────────────────────────────────────────
            stats_.lidar_total++;

            // Rate reduction: skip frames that don't fall on the divisor.
            if (fault_.lidar_rate_divisor > 1) {
                if ((lidar_seq_ % fault_.lidar_rate_divisor) != 0) {
                    lidar_seq_++;
                    stats_.lidar_decimated++;
                    continue;  // skip this frame, read next event
                }
            }

            // Random drop.
            if (fault_.lidar_drop_rate > 0.0) {
                if (drop_dist_(rng_) < fault_.lidar_drop_rate) {
                    lidar_seq_++;
                    stats_.lidar_dropped++;
                    continue;  // skip this frame
                }
            }

            // Timestamp jitter.
            if (fault_.timestamp_jitter_ns > 0) {
                ev->timestamp_ns = jitterTimestamp(ev->timestamp_ns);
                // Note: PointCloudFrame is const-shared, we jitter the
                // StreamEvent timestamp only (engine uses event timestamp).
                stats_.events_jittered++;
            }

            lidar_seq_++;
            stats_.lidar_delivered++;
            return ev;
        }
    }
}

std::vector<GtPose> FaultInjectorAdapter::loadGroundTruth() {
    // Ground truth is never corrupted — essential for accuracy evaluation.
    return inner_->loadGroundTruth();
}

void FaultInjectorAdapter::rewind() {
    inner_->rewind();
    lidar_seq_ = 0;
    stats_ = {};
    rng_.seed(fault_.seed);  // re-seed for reproducibility
}

size_t FaultInjectorAdapter::framesEmitted() const {
    return stats_.lidar_delivered;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Private helpers
// ═════════════════════════════════════════════════════════════════════════════

void FaultInjectorAdapter::injectImuNoise(ImuSample& s) {
    const double accel_sigma = fault_.imu_noise_scale * fault_.baseline_accel_sigma;
    const double gyro_sigma  = fault_.imu_noise_scale * fault_.baseline_gyro_sigma;

    s.accel[0] += accel_sigma * accel_noise_(rng_);
    s.accel[1] += accel_sigma * accel_noise_(rng_);
    s.accel[2] += accel_sigma * accel_noise_(rng_);

    s.gyro[0] += gyro_sigma * gyro_noise_(rng_);
    s.gyro[1] += gyro_sigma * gyro_noise_(rng_);
    s.gyro[2] += gyro_sigma * gyro_noise_(rng_);
}

int64_t FaultInjectorAdapter::jitterTimestamp(int64_t ts) {
    int64_t jitter = jitter_dist_(rng_);
    // Ensure timestamp doesn't go negative.
    return std::max(int64_t{0}, ts + jitter);
}

} // namespace eval
