// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Evaluation Framework: Fault Injection Adapter
// ─────────────────────────────────────────────────────────────────────────────
//
// Decorator around any DatasetAdapter that injects deterministic sensor
// faults for hardware-grade reliability validation.
//
// Supported fault modes:
//
//   1. LiDAR frame drop    – randomly drops a configurable fraction of scans
//   2. IMU noise scaling   – injects additive Gaussian noise on top of the
//                            baseline IMU signal (scales with multiplier)
//   3. Timestamp jitter    – adds uniform random jitter to all timestamps
//   4. LiDAR rate reduction – decimates LiDAR to 1/N of the original rate
//
// All faults are seeded for reproducibility.  The ground truth trajectory
// is always forwarded unmodified so that accuracy metrics remain meaningful.
//
// Usage:
//
//   FaultConfig fault;
//   fault.lidar_drop_rate   = 0.10;        // drop 10 % of scans
//   fault.imu_noise_scale   = 2.0;         // double the IMU noise floor
//   fault.timestamp_jitter_ns = 500'000;   // ±0.5 ms uniform jitter
//   fault.lidar_rate_divisor  = 2;         // halve LiDAR frequency
//
//   auto inner = createAdapter("aggressive_drone", false, {});
//   inner->open("aggressive_drone", 0);
//   auto faulty = std::make_unique<FaultInjectorAdapter>(std::move(inner), fault);
//   // faulty is now a DatasetAdapter with faults applied
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "eval/dataset_adapter.h"

#include <cstdint>
#include <memory>
#include <random>
#include <string>

namespace eval {

// ═════════════════════════════════════════════════════════════════════════════
//  FaultConfig — what faults to inject and at what severity
// ═════════════════════════════════════════════════════════════════════════════

struct FaultConfig {
    // ── LiDAR frame drop ────────────────────────────────────────────────
    /// Probability of dropping each LiDAR frame (0.0 = none, 1.0 = all).
    double lidar_drop_rate{0.0};

    // ── Additive IMU noise ──────────────────────────────────────────────
    /// Extra noise multiplier on top of baseline.
    /// 1.0 = double the noise floor, 0.0 = no extra noise.
    /// The injected noise sigma = imu_noise_scale × baseline_sigma.
    double imu_noise_scale{0.0};

    /// Baseline sigmas used for scaling (if 0, sensible defaults are used).
    double baseline_accel_sigma{0.02};   // m/s²
    double baseline_gyro_sigma{0.002};   // rad/s

    // ── Timestamp jitter ────────────────────────────────────────────────
    /// Maximum timestamp jitter in nanoseconds (±).
    /// Jitter is drawn from uniform distribution [-jitter, +jitter].
    int64_t timestamp_jitter_ns{0};

    // ── LiDAR rate reduction ────────────────────────────────────────────
    /// Keep every Nth LiDAR frame.  1 = keep all, 2 = 50 % rate, etc.
    int lidar_rate_divisor{1};

    // ── RNG seed ────────────────────────────────────────────────────────
    uint64_t seed{42};

    /// Human label for this fault configuration.
    std::string label() const;

    /// True if any fault is active.
    bool active() const;
};

// ═════════════════════════════════════════════════════════════════════════════
//  FaultInjectorAdapter — decorator that applies faults
// ═════════════════════════════════════════════════════════════════════════════

class FaultInjectorAdapter final : public DatasetAdapter {
public:
    /// @param inner  The real adapter (takes ownership).
    /// @param fault  Fault configuration.
    FaultInjectorAdapter(std::unique_ptr<DatasetAdapter> inner, FaultConfig fault);

    // ── DatasetAdapter interface (forwarded with fault injection) ────────
    bool open(const std::string& path, size_t max_frames = 0) override;
    DatasetInfo info() const override;
    std::optional<StreamEvent> next() override;
    std::vector<GtPose> loadGroundTruth() override;
    void rewind() override;
    size_t framesEmitted() const override;

    // ── Fault statistics (for reporting) ────────────────────────────────
    struct FaultStats {
        size_t lidar_total{0};      ///< Total LiDAR frames from inner adapter
        size_t lidar_dropped{0};    ///< Frames dropped by fault injector
        size_t lidar_decimated{0};  ///< Frames skipped by rate reduction
        size_t lidar_delivered{0};  ///< Frames actually delivered downstream
        size_t imu_total{0};        ///< Total IMU samples processed
        size_t events_jittered{0};  ///< Events with timestamp jitter applied
    };

    const FaultStats& faultStats() const { return stats_; }
    const FaultConfig& faultConfig() const { return fault_; }

private:
    std::unique_ptr<DatasetAdapter> inner_;
    FaultConfig fault_;

    std::mt19937_64 rng_;
    std::uniform_real_distribution<double> drop_dist_{0.0, 1.0};
    std::normal_distribution<double> accel_noise_{0.0, 1.0};
    std::normal_distribution<double> gyro_noise_{0.0, 1.0};
    std::uniform_int_distribution<int64_t> jitter_dist_;

    size_t lidar_seq_{0};   // LiDAR sequence counter (for rate division)
    FaultStats stats_;

    /// Apply IMU noise injection to a sample (in-place).
    void injectImuNoise(ImuSample& s);

    /// Apply timestamp jitter (returns modified timestamp).
    int64_t jitterTimestamp(int64_t ts);
};

} // namespace eval
