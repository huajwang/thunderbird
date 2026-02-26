// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Health Monitor utilities
// ─────────────────────────────────────────────────────────────────────────────
//
// Helper functions for human-readable fault reporting and snapshot
// serialisation.  The core SlamHealthMonitor is header-only for inlining
// on the hot path.
//
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/odom/slam_health.h"

#include <cstdio>
#include <sstream>

namespace thunderbird::odom {

// ═════════════════════════════════════════════════════════════════════════════
//  Fault flag stringification
// ═════════════════════════════════════════════════════════════════════════════

std::string fault_flags_to_string(FaultFlags flags) {
    if (flags == FaultFlags::None) return "None";

    std::string result;
    auto append = [&](FaultFlags f, const char* name) {
        if (has_fault(flags, f)) {
            if (!result.empty()) result += " | ";
            result += name;
        }
    };

    append(FaultFlags::ImuDropout,     "ImuDropout");
    append(FaultFlags::LidarDropout,   "LidarDropout");
    append(FaultFlags::DegenerateGeom, "DegenerateGeom");
    append(FaultFlags::EkfDivergence,  "EkfDivergence");
    append(FaultFlags::HighDriftRate,  "HighDriftRate");
    append(FaultFlags::ResidualSpike,  "ResidualSpike");
    append(FaultFlags::ImuSaturation,  "ImuSaturation");
    append(FaultFlags::CovarianceNaN,  "CovarianceNaN");
    append(FaultFlags::MapDegraded,    "MapDegraded");
    append(FaultFlags::TimeSyncLost,   "TimeSyncLost");

    return result;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Snapshot → JSON
// ═════════════════════════════════════════════════════════════════════════════

std::string health_snapshot_to_json(const SlamHealthSnapshot& s) {
    std::ostringstream os;
    os << "{\n";
    os << "  \"timestamp_ns\": " << s.timestamp_ns << ",\n";
    os << "  \"state\": \"" << health_state_name(s.state) << "\",\n";
    os << "  \"faults\": \"" << fault_flags_to_string(s.active_faults) << "\",\n";
    os << "  \"fault_bits\": " << static_cast<uint16_t>(s.active_faults) << ",\n";
    os << "  \"validity\": \"" << output_validity_name(s.validity) << "\",\n";
    os << "  \"confidence\": " << s.confidence << ",\n";

    os << "  \"scores\": {\n";
    os << "    \"imu\": " << s.imu_score << ",\n";
    os << "    \"lidar\": " << s.lidar_score << ",\n";
    os << "    \"geometry\": " << s.geometry_score << ",\n";
    os << "    \"ekf\": " << s.ekf_score << ",\n";
    os << "    \"drift\": " << s.drift_score << ",\n";
    os << "    \"residual\": " << s.residual_score << "\n";
    os << "  },\n";

    os << "  \"timing\": {\n";
    os << "    \"last_imu_ns\": " << s.last_imu_ns << ",\n";
    os << "    \"last_lidar_ns\": " << s.last_lidar_ns << ",\n";
    os << "    \"imu_gap_ns\": " << s.imu_gap_ns << ",\n";
    os << "    \"lidar_gap_ns\": " << s.lidar_gap_ns << "\n";
    os << "  },\n";

    os << "  \"diagnostics\": {\n";
    os << "    \"cov_trace\": " << s.cov_trace << ",\n";
    os << "    \"residual_ema\": " << s.residual_ema << ",\n";
    os << "    \"drift_speed_mps\": " << s.drift_speed_mps << ",\n";
    os << "    \"eigenvalue_ratio\": " << s.eigenvalue_ratio << "\n";
    os << "  },\n";

    os << "  \"counters\": {\n";
    os << "    \"total_imu_dropouts\": " << s.total_imu_dropouts << ",\n";
    os << "    \"total_lidar_dropouts\": " << s.total_lidar_dropouts << ",\n";
    os << "    \"total_ekf_divergences\": " << s.total_ekf_divergences << ",\n";
    os << "    \"total_state_transitions\": " << s.total_state_transitions << ",\n";
    os << "    \"time_in_state_ns\": " << s.time_in_state_ns << "\n";
    os << "  }\n";

    os << "}\n";
    return os.str();
}

// ═════════════════════════════════════════════════════════════════════════════
//  Snapshot → human-readable text
// ═════════════════════════════════════════════════════════════════════════════

std::string health_snapshot_to_text(const SlamHealthSnapshot& s) {
    char buf[1024];
    std::snprintf(buf, sizeof(buf),
        "SLAM Health: %s  confidence=%.2f  validity=%s\n"
        "  Faults: %s\n"
        "  Scores: imu=%.2f lidar=%.2f geom=%.2f ekf=%.2f drift=%.2f resid=%.2f\n"
        "  IMU gap: %.1f ms  LiDAR gap: %.1f ms\n"
        "  Cov trace: %.3f  Residual EMA: %.4f  Drift: %.2f m/s\n"
        "  Dropouts: imu=%lu lidar=%lu  Divergences: %lu  Transitions: %lu\n",
        health_state_name(s.state),
        static_cast<double>(s.confidence),
        output_validity_name(s.validity),
        fault_flags_to_string(s.active_faults).c_str(),
        static_cast<double>(s.imu_score),
        static_cast<double>(s.lidar_score),
        static_cast<double>(s.geometry_score),
        static_cast<double>(s.ekf_score),
        static_cast<double>(s.drift_score),
        static_cast<double>(s.residual_score),
        s.imu_gap_ns / 1.0e6,
        s.lidar_gap_ns / 1.0e6,
        s.cov_trace,
        s.residual_ema,
        s.drift_speed_mps,
        static_cast<unsigned long>(s.total_imu_dropouts),
        static_cast<unsigned long>(s.total_lidar_dropouts),
        static_cast<unsigned long>(s.total_ekf_divergences),
        static_cast<unsigned long>(s.total_state_transitions));
    return buf;
}

} // namespace thunderbird::odom
