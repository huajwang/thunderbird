// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Evaluation Framework: Stress-Test Adapter
// ─────────────────────────────────────────────────────────────────────────────
//
// Generates extreme synthetic sensor sequences to push SLAM engines beyond
// normal operating conditions.  Produces both LiDAR and IMU events with
// known ground truth, enabling fully deterministic stress testing without
// real hardware data.
//
// Presets:
//
//   aggressive_drone  - High angular velocity (up to 6 rad/s), sudden direction
//                       changes, altitude variation, sparse feature zones.
//   fast_car          - 120 km/h linear + tight turns, large point counts,
//                       simulated urban canyon with degenerate planes.
//   degenerate_corridor - Long straight corridors with minimal geometric
//                       features (planar walls only), low point density,
//                       tests LiDAR degeneracy handling.
//   spinning_top      - Pure rotation in place (translation ≈ 0). Extreme
//                       angular rates, stress-tests IMU integration.
//   custom            - User-specified parameters via StressConfig.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "eval/dataset_adapter.h"
#include "eval/imu_interpolator.h"

#include <array>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace eval {

// ═════════════════════════════════════════════════════════════════════════════
//  StressConfig — configurable stress-test parameters
// ═════════════════════════════════════════════════════════════════════════════

struct StressConfig {
    // ── Trajectory ──────────────────────────────────────────────────────
    double duration_s{30.0};             ///< Total sequence duration
    double lidar_rate_hz{10.0};          ///< LiDAR frame rate
    double imu_rate_hz{200.0};           ///< IMU sample rate

    // ── Motion profile ──────────────────────────────────────────────────
    double max_linear_speed_mps{5.0};    ///< m/s peak linear speed
    double max_angular_rate_rps{2.0};    ///< rad/s peak angular rate
    double jerk_factor{1.0};             ///< 1.0 = smooth, >1.0 = aggressive jerks
    double direction_change_freq{0.5};   ///< Hz of random direction changes

    // ── LiDAR point cloud ───────────────────────────────────────────────
    size_t min_points{500};              ///< Minimum points per scan
    size_t max_points{50000};            ///< Maximum points per scan
    double feature_density{1.0};         ///< 0.1 = sparse, 1.0 = normal
    double max_range_m{80.0};            ///< LiDAR maximum range

    // ── Noise ───────────────────────────────────────────────────────────
    double accel_noise_sigma{0.02};      ///< m/s² IMU accel noise
    double gyro_noise_sigma{0.002};      ///< rad/s IMU gyro noise
    double lidar_noise_sigma{0.01};      ///< metres, point cloud noise

    // ── RNG ─────────────────────────────────────────────────────────────
    uint64_t seed{12345};                ///< RNG seed for reproducibility

    // ── Named presets ───────────────────────────────────────────────────
    static StressConfig aggressiveDrone();
    static StressConfig fastCar();
    static StressConfig degenerateCorridor();
    static StressConfig spinningTop();
};

// ═════════════════════════════════════════════════════════════════════════════
//  StressTestAdapter
// ═════════════════════════════════════════════════════════════════════════════

class StressTestAdapter final : public DatasetAdapter {
public:
    explicit StressTestAdapter(StressConfig cfg = {});

    bool open(const std::string& path, size_t max_frames = 0) override;
    DatasetInfo info() const override;
    std::optional<StreamEvent> next() override;
    std::vector<GtPose> loadGroundTruth() override;
    void rewind() override;
    size_t framesEmitted() const override { return lidar_emitted_; }

private:
    StressConfig cfg_;

    // ── Pre-generated trajectory ────────────────────────────────────────
    std::vector<GtPose>   gt_poses_;       // GT at LiDAR timestamps
    std::vector<ImuSample> imu_samples_;   // All synthetic IMU

    // ── Playback state ──────────────────────────────────────────────────
    size_t lidar_cursor_{0};
    size_t imu_cursor_{0};
    size_t lidar_emitted_{0};
    size_t max_frames_{0};
    std::string preset_name_;

    std::mt19937 rng_;

    // ── Trajectory generation ───────────────────────────────────────────
    void generateTrajectory();
    std::shared_ptr<const PointCloudFrame> generateCloud(
        size_t frame_idx, int64_t timestamp_ns, const GtPose& pose);
};

/// Factory function.
std::unique_ptr<DatasetAdapter> make_stress_adapter(StressConfig cfg = {});

} // namespace eval
