// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM / Odometry data types (public API)
// ─────────────────────────────────────────────────────────────────────────────
//
// These types form the public contract between the OdometryManager and user
// code.  They are intentionally **Eigen-free** so that the public header
// never pulls in Eigen, keeping the ABI stable across Eigen versions and
// eliminating alignment-macro hazards in user compilation units.
//
// Design principles (aligned with existing SDK conventions):
//
//   • Timestamps are **int64_t nanoseconds** (same as types.h, sensor_data.h).
//
//   • Poses use **double precision** for position, velocity, and bias;
//     quaternions are stored as double[4] in Hamilton convention [w,x,y,z].
//
//   • All structs are standard-layout where possible so they can be
//     memcpy'd, mmap'd, or serialized with trivial binary I/O.
//
//   • No Eigen, no PCL, no OpenCV in this header.  Internal code converts
//     to/from Eigen types inside the PImpl boundary.
//
//   • Shared ownership uses `shared_ptr<const T>` for zero-copy fan-out
//     (same pattern as LidarFrame, CameraFrame in the sensor pipeline).
//
//   • Every struct documents its alignment and padding explicitly.
//
// Thread safety:
//   • All types are **immutable after construction** when published via
//     shared_ptr<const T>.  Mutation is only permitted inside the
//     OdometryManager's internal threads before publication.
//   • Callback delivery happens on a dedicated OutputDispatcher thread;
//     user callbacks must not block or modify the pointed-to data.
//   • Pull API methods return shared_ptr copies (atomic ref-count bump).
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <vector>

namespace thunderbird::odom {

// ═════════════════════════════════════════════════════════════════════════════
//  Constants
// ═════════════════════════════════════════════════════════════════════════════

/// Error-state dimension used in the ESIKF (rotation 3 + pos 3 + vel 3 +
/// gyro bias 3 + accel bias 3 + gravity 3 + extrinsic rot 3 + extrinsic pos 3).
static constexpr int kStateDim = 24;

/// Covariance matrix size (row-major, kStateDim × kStateDim).
static constexpr int kCovSize  = kStateDim * kStateDim;

// ═════════════════════════════════════════════════════════════════════════════
//  Tracking status
// ═════════════════════════════════════════════════════════════════════════════

/// Current odometry tracking state, exposed via OdometryManager::status().
enum class TrackingStatus : uint8_t {
    Initializing = 0,   ///< Collecting IMU samples for gravity alignment
    Converging   = 1,   ///< First scans — covariance still large
    Tracking     = 2,   ///< Nominal operation
    Degraded     = 3,   ///< High residuals or few correspondences
    Lost         = 4,   ///< Diverged — reset recommended
};

/// Human-readable name for a TrackingStatus value.
inline const char* tracking_status_name(TrackingStatus s) noexcept {
    switch (s) {
        case TrackingStatus::Initializing: return "Initializing";
        case TrackingStatus::Converging:   return "Converging";
        case TrackingStatus::Tracking:     return "Tracking";
        case TrackingStatus::Degraded:     return "Degraded";
        case TrackingStatus::Lost:         return "Lost";
    }
    return "Unknown";
}

// ═════════════════════════════════════════════════════════════════════════════
//  ImuSample  —  single inertial measurement (public SLAM variant)
// ═════════════════════════════════════════════════════════════════════════════
//
// Mirrors thunderbird::ImuSample from types.h but uses double precision for
// the estimator accumulation path.  The IO layer converts float→double at
// ingress so the estimation core never touches float IMU data.
//
// Memory layout (64 bytes, naturally aligned on 8-byte boundary):
//   offset  0:  timestamp_ns     (8)
//   offset  8:  accel[0..2]      (24)
//   offset 32:  gyro[0..2]       (24)
//   offset 56:  temperature      (8)
//   total: 64 bytes — no padding
// ─────────────────────────────────────────────────────────────────────────────

struct ImuSample {
    int64_t              timestamp_ns{0};   ///< hardware timestamp (ns)
    std::array<double,3> accel{};           ///< m/s²  (x, y, z)
    std::array<double,3> gyro{};            ///< rad/s (x, y, z)
    double               temperature{0.0};  ///< °C
};

static_assert(sizeof(ImuSample) == 64, "ImuSample must be 64 bytes (no padding)");
static_assert(alignof(ImuSample) == 8, "ImuSample must be 8-byte aligned");

// ═════════════════════════════════════════════════════════════════════════════
//  PointXYZIT  —  single LiDAR return with per-point timestamp
// ═════════════════════════════════════════════════════════════════════════════
//
// Identical layout to thunderbird::data::PointXYZIT so zero-copy reinterpret
// is valid (both are standard-layout, same field sequence).  Kept in the
// odom namespace to avoid ambiguity in files that include both headers.
//
// Memory layout (20 bytes):
//   offset  0: x         (4)
//   offset  4: y         (4)
//   offset  8: z         (4)
//   offset 12: intensity (4)
//   offset 16: dt_ns     (4)
//   total: 20 bytes — no padding
// ─────────────────────────────────────────────────────────────────────────────

struct PointXYZIT {
    float   x{0};          ///< metres, body frame
    float   y{0};          ///< metres, body frame
    float   z{0};          ///< metres, body frame
    float   intensity{0};  ///< normalised 0–255
    int32_t dt_ns{0};      ///< ns offset from parent frame timestamp
};

static_assert(sizeof(PointXYZIT)  == 20, "PointXYZIT must be 20 bytes");
static_assert(alignof(PointXYZIT) ==  4, "PointXYZIT must be 4-byte aligned");

// ═════════════════════════════════════════════════════════════════════════════
//  PointCloudFrame  —  one LiDAR sweep (raw or deskewed)
// ═════════════════════════════════════════════════════════════════════════════
//
// Owns the point buffer.  After publication via shared_ptr<const>, the
// vector is immutable and can be read concurrently from any thread.
//
// The `is_deskewed` flag distinguishes raw scans (from the sensor) from
// motion-compensated clouds (output by PointDeskewer).
// ─────────────────────────────────────────────────────────────────────────────

struct PointCloudFrame {
    int64_t  timestamp_ns{0};        ///< scan-start hardware timestamp (ns)
    uint32_t sequence{0};            ///< monotonic frame counter
    bool     is_deskewed{false};     ///< true if motion-compensated

    std::vector<PointXYZIT> points;  ///< point data

    /// Number of points.
    [[nodiscard]] uint32_t num_points() const noexcept {
        return static_cast<uint32_t>(points.size());
    }

    /// Zero-copy span over points.
    [[nodiscard]] std::span<const PointXYZIT> point_span() const noexcept {
        return {points.data(), points.size()};
    }

    /// Total byte size of the point payload (for serialization / allocation).
    [[nodiscard]] size_t point_bytes() const noexcept {
        return points.size() * sizeof(PointXYZIT);
    }
};

// ═════════════════════════════════════════════════════════════════════════════
//  Pose6D  —  full 6-DOF pose with velocity, biases, and covariance
// ═════════════════════════════════════════════════════════════════════════════
//
// This is the primary output of the odometry pipeline, published at IMU rate
// (200 Hz) for propagated poses and at LiDAR rate (10 Hz) for corrected poses.
//
// Quaternion convention: Hamilton, [w, x, y, z], right-handed.
//   q represents the rotation from body frame to world frame:
//     p_world = q ⊗ p_body ⊗ q*
//
// All arrays are double precision for numerical stability in downstream
// consumers (path planning, control loops).
//
// Covariance is the marginal 6×6 block [rotation(3), position(3)] stored
// row-major.  Full 24×24 covariance is available via query API if needed.
//
// Memory layout (fixed-size portion):
//   timestamp_ns    8 bytes
//   quaternion     32 bytes  (4 × double)
//   position       24 bytes  (3 × double)
//   velocity       24 bytes  (3 × double)
//   gyro_bias      24 bytes  (3 × double)
//   accel_bias     24 bytes  (3 × double)
//   gravity        24 bytes  (3 × double)
//   covariance_6x6 288 bytes (36 × double)
//   tracking_status 1 byte
//   is_corrected    1 byte
//   _pad            6 bytes  (to align total to 8-byte boundary)
//   ─────────────────────────
//   total:        456 bytes
// ─────────────────────────────────────────────────────────────────────────────

struct Pose6D {
    int64_t              timestamp_ns{0};        ///< ns, same clock as sensor data

    // ── Orientation ─────────────────────────────────────────────────────
    std::array<double,4> quaternion{1,0,0,0};    ///< [w,x,y,z] body→world

    // ── Translation & dynamics ──────────────────────────────────────────
    std::array<double,3> position{};             ///< metres, world frame
    std::array<double,3> velocity{};             ///< m/s, world frame

    // ── Estimator biases ────────────────────────────────────────────────
    std::array<double,3> gyro_bias{};            ///< rad/s
    std::array<double,3> accel_bias{};           ///< m/s²

    // ── Gravity estimate ────────────────────────────────────────────────
    std::array<double,3> gravity{0, 0, -9.81};   ///< m/s², world frame

    // ── Uncertainty: marginal 6×6 covariance [rot(3), pos(3)] ───────────
    /// Row-major.  Index: cov[row * 6 + col].
    /// Rotation block: rows 0–2, cols 0–2 (rad²).
    /// Position block: rows 3–5, cols 3–5 (m²).
    /// Cross terms:    rows 0–2, cols 3–5 and transpose.
    std::array<double,36> covariance_6x6{};

    // ── Status ──────────────────────────────────────────────────────────
    TrackingStatus tracking_status{TrackingStatus::Initializing};

    /// True if this pose includes an ESIKF measurement update (LiDAR or
    /// visual).  False for IMU-only propagated poses.
    bool is_corrected{false};

    // ── Helpers ─────────────────────────────────────────────────────────

    /// Timestamp as floating-point seconds.
    [[nodiscard]] double to_seconds() const noexcept {
        return timestamp_ns / 1.0e9;
    }

    /// Euler angles (roll, pitch, yaw) in radians — convenience only.
    /// For precision-critical paths, use the quaternion directly.
    [[nodiscard]] std::array<double,3> euler_rpy() const noexcept {
        const double w = quaternion[0], x = quaternion[1],
                     y = quaternion[2], z = quaternion[3];
        // roll  (x-axis)
        const double sinr_cosp = 2.0 * (w*x + y*z);
        const double cosr_cosp = 1.0 - 2.0 * (x*x + y*y);
        const double roll = std::atan2(sinr_cosp, cosr_cosp);
        // pitch (y-axis)
        const double sinp = 2.0 * (w*y - z*x);
        const double pitch = (std::abs(sinp) >= 1.0)
            ? std::copysign(3.14159265358979323846 / 2.0, sinp)
            : std::asin(sinp);
        // yaw   (z-axis)
        const double siny_cosp = 2.0 * (w*z + x*y);
        const double cosy_cosp = 1.0 - 2.0 * (y*y + z*z);
        const double yaw = std::atan2(siny_cosp, cosy_cosp);
        return {roll, pitch, yaw};
    }
};

// ═════════════════════════════════════════════════════════════════════════════
//  SlamOutput  —  composite output bundle per LiDAR scan
// ═════════════════════════════════════════════════════════════════════════════
//
// Emitted once per LiDAR scan (10 Hz) after the ESIKF update completes.
// Bundles the corrected pose, the deskewed cloud, and map statistics.
//
// Published as shared_ptr<const SlamOutput> — immutable after creation.
// ─────────────────────────────────────────────────────────────────────────────

struct LocalMapInfo {
    size_t               total_points{0};    ///< current ikd-Tree size
    std::array<double,3> center{};           ///< world-frame map center
    double               radius{0.0};        ///< bounding sphere radius (m)
};

struct SlamOutput {
    int64_t timestamp_ns{0};     ///< scan timestamp this output corresponds to

    // ── Corrected pose at scan time ─────────────────────────────────────
    Pose6D  pose;

    // ── Motion-compensated point cloud ──────────────────────────────────
    /// Shared to avoid copying the potentially large point buffer.
    std::shared_ptr<const PointCloudFrame> deskewed_cloud;

    // ── Map snapshot ────────────────────────────────────────────────────
    LocalMapInfo map_info;

    // ── Estimator diagnostics ───────────────────────────────────────────
    int      esikf_iterations{0};     ///< iterations this update took
    double   esikf_residual{0.0};     ///< final point-to-plane residual
    uint32_t correspondences{0};      ///< points matched to map
    double   update_latency_ms{0.0};  ///< wall-clock time for this update
};

// ═════════════════════════════════════════════════════════════════════════════
//  OdometryStats  —  cumulative pipeline diagnostics
// ═════════════════════════════════════════════════════════════════════════════

struct OdometryStats {
    uint64_t imu_samples_processed{0};
    uint64_t lidar_scans_processed{0};
    uint64_t visual_frames_processed{0};

    double   avg_imu_propagation_us{0.0};  ///< mean IMU propagation time (µs)
    double   avg_esikf_update_ms{0.0};     ///< mean ESIKF update time (ms)
    double   avg_visual_update_ms{0.0};    ///< mean visual update time (ms)

    size_t   map_points{0};                ///< current ikd-Tree size
    size_t   imu_queue_drops{0};           ///< ingress ring buffer drops
    size_t   lidar_queue_drops{0};
    size_t   image_queue_drops{0};
};

// ═════════════════════════════════════════════════════════════════════════════
//  Callback typedefs
// ═════════════════════════════════════════════════════════════════════════════

/// 200 Hz — every IMU propagation or ESIKF correction.
using PoseCallback          = std::function<void(std::shared_ptr<const Pose6D>)>;

/// 10 Hz — one per LiDAR scan, after ESIKF update.
using DeskewedCloudCallback = std::function<void(std::shared_ptr<const PointCloudFrame>)>;

/// 10 Hz — full composite output per scan.
using SlamOutputCallback    = std::function<void(std::shared_ptr<const SlamOutput>)>;

/// On every map insertion (10 Hz, may be throttled).
using MapUpdateCallback     = std::function<void(const LocalMapInfo&)>;

// ═════════════════════════════════════════════════════════════════════════════
//  Serialization: Binary Record Layout
// ═════════════════════════════════════════════════════════════════════════════
//
// All SLAM types support trivial binary serialization for recording and
// replay.  The strategy follows the existing recording_format.h pattern:
//
// ┌─────────────────────────────────────────────────────────────────┐
// │ RecordHeader (from recording_format.h)                         │
// │   sensor_type = ODOM_POSE / ODOM_CLOUD / ODOM_SLAM_OUTPUT      │
// │   timestamp_ns, payload_length                                  │
// ├─────────────────────────────────────────────────────────────────┤
// │ Payload (one of):                                               │
// │                                                                 │
// │ ── Pose6D record ────────────────────────────────────────────── │
// │   Pose6DRecord (POD, packed)                                    │
// │   Fixed 456 bytes — single fwrite                               │
// │                                                                 │
// │ ── PointCloudFrame record ───────────────────────────────────── │
// │   CloudRecordHeader: timestamp_ns(8) + sequence(4) +            │
// │     is_deskewed(1) + pad(3) + num_points(4) = 20 bytes          │
// │   PointXYZIT[num_points]: 20 × N bytes                          │
// │                                                                 │
// │ ── SlamOutput record ────────────────────────────────────────── │
// │   SlamOutputRecordHeader:                                       │
// │     timestamp_ns(8) + Pose6DRecord(456) + LocalMapInfo(40) +    │
// │     esikf_iterations(4) + esikf_residual(8) +                   │
// │     correspondences(4) + update_latency_ms(8) +                 │
// │     num_points(4) = 532 bytes                                   │
// │   PointXYZIT[num_points]: 20 × N bytes (deskewed cloud)         │
// └─────────────────────────────────────────────────────────────────┘
//
// Reader strategy:
//   1. Read RecordHeader → know sensor_type + payload_length
//   2. Read fixed sub-header → know variable array size
//   3. Read variable array → reconstruct struct
//
// All multi-byte fields are little-endian (same as recording_format.h).
// ─────────────────────────────────────────────────────────────────────────────

#pragma pack(push, 1)

/// Packed binary representation of Pose6D for disk I/O.
/// Mirrors Pose6D field order — convert with memcpy or field-by-field copy.
struct Pose6DRecord {
    int64_t  timestamp_ns;
    double   quaternion[4];   // w, x, y, z
    double   position[3];
    double   velocity[3];
    double   gyro_bias[3];
    double   accel_bias[3];
    double   gravity[3];
    double   covariance_6x6[36];
    uint8_t  tracking_status;
    uint8_t  is_corrected;
    uint8_t  _pad[6];         // align to 8-byte boundary
};

static_assert(sizeof(Pose6DRecord) == 456, "Pose6DRecord must be 456 bytes");

/// Packed header preceding a PointCloudFrame payload on disk.
struct CloudRecordHeader {
    int64_t  timestamp_ns;
    uint32_t sequence;
    uint8_t  is_deskewed;
    uint8_t  _pad[3];
    uint32_t num_points;      // followed by num_points × PointXYZIT
};

static_assert(sizeof(CloudRecordHeader) == 20, "CloudRecordHeader must be 20 bytes");

/// Packed header for a full SlamOutput record.
struct SlamOutputRecordHeader {
    int64_t       timestamp_ns;
    Pose6DRecord  pose;
    // LocalMapInfo (packed inline)
    uint64_t      map_total_points;
    double        map_center[3];
    double        map_radius;
    // Diagnostics
    int32_t       esikf_iterations;
    double        esikf_residual;
    uint32_t      correspondences;
    double        update_latency_ms;
    // Cloud follows
    uint32_t      num_cloud_points;  // followed by num × PointXYZIT
    uint8_t       _pad[4];           // align to 8-byte boundary
};

static_assert(sizeof(SlamOutputRecordHeader) == 536,
              "SlamOutputRecordHeader must be 536 bytes");

#pragma pack(pop)

// ═════════════════════════════════════════════════════════════════════════════
//  Conversion helpers (Pose6D ↔ Pose6DRecord)
// ═════════════════════════════════════════════════════════════════════════════

/// Convert a Pose6D to its packed binary representation.
inline Pose6DRecord to_record(const Pose6D& p) noexcept {
    Pose6DRecord r{};
    r.timestamp_ns = p.timestamp_ns;
    for (int i = 0; i < 4; ++i) r.quaternion[i]   = p.quaternion[i];
    for (int i = 0; i < 3; ++i) r.position[i]     = p.position[i];
    for (int i = 0; i < 3; ++i) r.velocity[i]     = p.velocity[i];
    for (int i = 0; i < 3; ++i) r.gyro_bias[i]    = p.gyro_bias[i];
    for (int i = 0; i < 3; ++i) r.accel_bias[i]   = p.accel_bias[i];
    for (int i = 0; i < 3; ++i) r.gravity[i]      = p.gravity[i];
    for (int i = 0; i < 36;++i) r.covariance_6x6[i] = p.covariance_6x6[i];
    r.tracking_status = static_cast<uint8_t>(p.tracking_status);
    r.is_corrected    = p.is_corrected ? 1 : 0;
    return r;
}

/// Convert a packed binary record back to Pose6D.
inline Pose6D from_record(const Pose6DRecord& r) noexcept {
    Pose6D p{};
    p.timestamp_ns = r.timestamp_ns;
    for (int i = 0; i < 4; ++i) p.quaternion[i]   = r.quaternion[i];
    for (int i = 0; i < 3; ++i) p.position[i]     = r.position[i];
    for (int i = 0; i < 3; ++i) p.velocity[i]     = r.velocity[i];
    for (int i = 0; i < 3; ++i) p.gyro_bias[i]    = r.gyro_bias[i];
    for (int i = 0; i < 3; ++i) p.accel_bias[i]   = r.accel_bias[i];
    for (int i = 0; i < 3; ++i) p.gravity[i]      = r.gravity[i];
    for (int i = 0; i < 36;++i) p.covariance_6x6[i] = r.covariance_6x6[i];
    p.tracking_status = static_cast<TrackingStatus>(r.tracking_status);
    p.is_corrected    = r.is_corrected != 0;
    return p;
}

} // namespace thunderbird::odom
