// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — acme_slamd: SLAM Daemon Process Manager
// ─────────────────────────────────────────────────────────────────────────────
//
// This header defines the SlamDaemon class — the top-level orchestrator for
// the acme_slamd process.  It wires together:
//
//   • DeviceManager (sensor ingress)
//   • AcmeSlamEngine (ESIKF + ikd-Tree estimation)
//   • IPC publisher (shared-memory pose + map snapshot)
//   • Health monitor (CPU, drift, diagnostics)
//   • Crash recovery (state checkpoint + graceful restart)
//   • Logging (structured, levelled, journald-compatible)
//
// ═══════════════════════════════════════════════════════════════════════════
//  Process Architecture
// ═══════════════════════════════════════════════════════════════════════════
//
//  ┌──────────────────────────────────────────────────────────────────────┐
//  │                        acme_slamd process                            │
//  │                                                                      │
//  │  ┌──────────────────────────────────────────────────────────────┐    │
//  │  │  main()                                                      │    │
//  │  │   ├─ parse CLI args                                          │    │
//  │  │   ├─ load config.yaml                                        │    │
//  │  │   ├─ install signal handlers (SIGTERM, SIGINT, SIGHUP)       │    │
//  │  │   ├─ construct SlamDaemon                                    │    │
//  │  │   ├─ daemon.start()                                          │    │
//  │  │   ├─ sd_notify(READY=1)  [systemd watchdog]                  │    │
//  │  │   └─ daemon.run()  [blocks until shutdown]                   │    │
//  │  └──────────────────────────────────────────────────────────────┘    │
//  │                                                                      │
//  │  Threads:                                                            │
//  │                                                                      │
//  │  ┌─────────────────────────┐                                         │
//  │  │  [1] DeviceManager      │  Sensor driver threads (LiDAR, IMU,     │
//  │  │      (internal threads)  │  Camera) — owned by DeviceManager.      │
//  │  └─────────┬───────────────┘                                         │
//  │            │ on_imu / on_lidar callbacks                             │
//  │            ▼                                                         │
//  │  ┌─────────────────────────┐                                         │
//  │  │  [2] AcmeSlamEngine     │  ESIKF worker thread —                  │
//  │  │      (worker thread)     │  propagate + update + publish.          │
//  │  └─────────┬───────────────┘                                         │
//  │            │ SlamOutput / Pose6D callbacks                           │
//  │            ▼                                                         │
//  │  ┌─────────────────────────┐  ┌─────────────────────────────┐       │
//  │  │  [3] IPC Publisher      │  │  [4] Health Monitor         │       │
//  │  │      thread              │  │      thread (1 Hz)          │       │
//  │  │                          │  │                              │       │
//  │  │  • Write Pose6D to SHM  │  │  • CPU usage                │       │
//  │  │  • Write map snapshot    │  │  • Drift estimate           │       │
//  │  │  • Wake ZMQ subscribers  │  │  • Queue drops              │       │
//  │  └─────────────────────────┘  │  • Restart safety            │       │
//  │                                │  • sd_notify(WATCHDOG=1)    │       │
//  │                                └─────────────────────────────┘       │
//  │                                                                      │
//  │  ┌─────────────────────────┐                                         │
//  │  │  [5] Checkpoint thread  │  Periodic state save for crash          │
//  │  │      (0.2 Hz = 5s)      │  recovery.  Writes ESIKF state +       │
//  │  │                          │  map digest to /var/lib/acme_slamd/    │
//  │  └─────────────────────────┘                                         │
//  │                                                                      │
//  └──────────────────────────────────────────────────────────────────────┘
//
// ═══════════════════════════════════════════════════════════════════════════
//  IPC Mechanism Choice
// ═══════════════════════════════════════════════════════════════════════════
//
//  Three options were evaluated:
//
//  ┌─────────────┬──────────────┬──────────────┬──────────────────────────┐
//  │ Mechanism    │ Latency      │ Complexity   │ Notes                    │
//  ├─────────────┼──────────────┼──────────────┼──────────────────────────┤
//  │ Shared mem   │ < 1 µs       │ Medium       │ Lowest latency, needs   │
//  │ (POSIX shm)  │              │              │ careful seqlock/fence.   │
//  │              │              │              │ Local-only.              │
//  ├─────────────┼──────────────┼──────────────┼──────────────────────────┤
//  │ ZeroMQ PUB   │ ~10–50 µs   │ Low          │ PUB/SUB, multi-language, │
//  │ (IPC/TCP)    │              │              │ no broker.  Good for     │
//  │              │              │              │ notification + payload.  │
//  ├─────────────┼──────────────┼──────────────┼──────────────────────────┤
//  │ gRPC         │ ~100–500 µs │ High         │ Schema evolution, TLS,   │
//  │              │              │              │ cross-network. Overkill  │
//  │              │              │              │ for local-only pose.     │
//  └─────────────┴──────────────┴──────────────┴──────────────────────────┘
//
//  Decision: **Shared memory (primary) + ZeroMQ PUB (notification)**
//
//  • Pose6D is written to a POSIX shared-memory segment via a seqlock.
//    Consumers mmap the segment and spin-read (< 1 µs).  This gives
//    control loops the lowest possible latency.
//
//  • A ZeroMQ PUB socket (ipc:///tmp/acme_slamd/pose) publishes a
//    lightweight notification (timestamp + sequence) whenever the SHM
//    is updated.  Subscribers that need to be woken (loggers, UI) listen
//    on SUB; latency-critical consumers (control) poll SHM directly.
//
//  • Map snapshots (large, ~10 MB) are published only on ZMQ PUB at
//    lower rate (1 Hz) as serialized binary.
//
//  • gRPC is NOT used — it adds a heavy dependency and latency penalty
//    unsuitable for the 200+ Hz pose output.  If remote access is needed,
//    a thin gRPC gateway can sit on top of the ZMQ stream.
//
// ═══════════════════════════════════════════════════════════════════════════
//  Crash Recovery Design
// ═══════════════════════════════════════════════════════════════════════════
//
//  Problem: if acme_slamd crashes or is OOM-killed, the SLAM state (ESIKF
//  covariance, ikd-Tree, bias estimates) is lost.  Restarting from scratch
//  requires a new gravity alignment phase and ~30 s to re-converge.
//
//  Solution: periodic state checkpointing + warm restart.
//
//  ┌─────────────────────────────────────────────────────────────────────┐
//  │  Checkpoint Strategy                                                │
//  │                                                                     │
//  │  Every 5 s (configurable), the checkpoint thread:                   │
//  │                                                                     │
//  │  1. Reads the latest SlamOutput from the engine (non-blocking).     │
//  │  2. Serializes:                                                     │
//  │     • Pose6DRecord (456 B) — full ESIKF state                       │
//  │     • IMU biases + gravity (48 B)                                   │
//  │     • Map digest: bounding box + point count (not full map)          │
//  │     • Timestamp + sequence + checksum (CRC32)                       │
//  │  3. Writes atomically to /var/lib/acme_slamd/checkpoint.bin:        │
//  │     • Write to checkpoint.tmp                                       │
//  │     • fsync()                                                       │
//  │     • rename(tmp → checkpoint.bin)  [atomic on POSIX]               │
//  │                                                                     │
//  │  On restart:                                                        │
//  │                                                                     │
//  │  1. Load checkpoint.bin, verify CRC32.                              │
//  │  2. Check timestamp: if stale (> 60 s old), discard → cold start.   │
//  │  3. If valid: pre-seed ESIKF state, biases, gravity.                │
//  │     Skip gravity alignment → enter Converging directly.             │
//  │  4. First 5 scans run with inflated covariance (×10) to allow       │
//  │     the estimator to re-localise against the map.                   │
//  │                                                                     │
//  │  Guarantees:                                                        │
//  │  • Warm restart converges in < 5 s (vs. 30 s cold).                 │
//  │  • Stale checkpoint (power loss > 60 s) → safe cold start.          │
//  │  • Corrupt checkpoint (bad CRC) → safe cold start.                  │
//  │  • Atomic rename prevents half-written files.                       │
//  └─────────────────────────────────────────────────────────────────────┘
//
// ═══════════════════════════════════════════════════════════════════════════
//  Logging Strategy
// ═══════════════════════════════════════════════════════════════════════════
//
//  acme_slamd uses structured, levelled logging compatible with both
//  journald (systemd) and file-based outputs.
//
//  Levels (syslog-compatible):
//    0 EMERG     — process cannot continue
//    3 ERR       — sensor disconnected, ESIKF diverged
//    4 WARNING   — high drift, queue drops, degraded tracking
//    6 INFO      — lifecycle events: start, stop, config reload
//    7 DEBUG     — per-scan stats, timing, queue depths
//
//  Outputs:
//    • stderr   — always (systemd captures to journal)
//    • file     — optional, /var/log/acme_slamd/slamd.log (rotation via
//                 logrotate or size-based, configurable)
//
//  Format:
//    [YYYY-MM-DD HH:MM:SS.mmm] [LEVEL] [module] message key=value ...
//
//  Example:
//    [2026-02-25 14:30:01.234] [INFO] [slam] scan=142 latency_ms=3.2
//      correspondences=412 status=Tracking drift_ns_per_sec=12400
//
//  Rate limiting:
//    Per-scan DEBUG messages are emitted at most once per 10 scans
//    to avoid flooding the journal when running at 20 Hz LiDAR.
//
//  Structured fields emitted with every log line:
//    • daemon=acme_slamd
//    • pid=<PID>
//    • uptime_s=<seconds since start>
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/odom/slam_engine.h"
#include "thunderbird/odom/slam_types.h"
#include "thunderbird/device_manager.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace thunderbird::slamd {

// ═════════════════════════════════════════════════════════════════════════════
//  Log levels (syslog-compatible)
// ═════════════════════════════════════════════════════════════════════════════

enum class LogLevel : uint8_t {
    Emerg   = 0,
    Alert   = 1,
    Crit    = 2,
    Err     = 3,
    Warning = 4,
    Notice  = 5,
    Info    = 6,
    Debug   = 7,
};

inline const char* log_level_name(LogLevel l) noexcept {
    switch (l) {
        case LogLevel::Emerg:   return "EMERG";
        case LogLevel::Alert:   return "ALERT";
        case LogLevel::Crit:    return "CRIT";
        case LogLevel::Err:     return "ERR";
        case LogLevel::Warning: return "WARNING";
        case LogLevel::Notice:  return "NOTICE";
        case LogLevel::Info:    return "INFO";
        case LogLevel::Debug:   return "DEBUG";
    }
    return "UNKNOWN";
}

// ═════════════════════════════════════════════════════════════════════════════
//  Daemon configuration (populated from config.yaml)
// ═════════════════════════════════════════════════════════════════════════════

/// Sensor connection settings.
struct SensorConfig {
    std::string device_uri;              ///< "eth://192.168.1.100:7500" or "usb://0"
    double      imu_rate_hz   = 400.0;
    double      lidar_rate_hz = 10.0;
};

/// IPC publisher settings.
struct IpcConfig {
    /// POSIX shared memory segment name for Pose6D.
    std::string shm_pose_name     = "/acme_slamd_pose";

    /// Size of the SHM segment (must fit Pose6D + seqlock header).
    size_t      shm_pose_bytes    = 4096;

    /// ZeroMQ PUB endpoint for pose notifications.
    std::string zmq_pose_endpoint = "ipc:///tmp/acme_slamd/pose";

    /// ZeroMQ PUB endpoint for map snapshots.
    std::string zmq_map_endpoint  = "ipc:///tmp/acme_slamd/map";

    /// Map snapshot publish rate (Hz).
    double      map_publish_hz    = 1.0;

    /// Enable ZMQ publisher (can be disabled for SHM-only mode).
    bool        enable_zmq        = true;

    /// Enable shared-memory publisher.
    bool        enable_shm        = true;
};

/// Health monitoring settings.
struct HealthConfig {
    /// Health check interval (Hz).
    double      health_check_hz        = 1.0;

    /// CPU usage warning threshold (0.0–1.0).
    double      cpu_warn_threshold     = 0.80;

    /// Clock drift warning threshold (ns/s).
    double      drift_warn_threshold   = 1'000'000.0;  // 1 ms/s

    /// Enable systemd watchdog notifications.
    bool        enable_watchdog        = true;

    /// Watchdog interval (µs).  Should be < WatchdogSec/2 in the unit file.
    uint64_t    watchdog_interval_us   = 5'000'000;  // 5 s
};

/// Crash recovery / checkpoint settings.
struct CheckpointConfig {
    /// Enable state checkpointing.
    bool        enable                 = true;

    /// Checkpoint directory.
    std::string directory              = "/var/lib/acme_slamd";

    /// Checkpoint filename.
    std::string filename               = "checkpoint.bin";

    /// Checkpoint interval (seconds).
    double      interval_s             = 5.0;

    /// Maximum checkpoint age for warm restart (seconds).
    double      max_age_s              = 60.0;

    /// Covariance inflation factor on warm restart.
    double      warm_restart_cov_scale = 10.0;
};

/// Logging settings.
struct LogConfig {
    /// Minimum log level.
    LogLevel    level                  = LogLevel::Info;

    /// Optional log file path (empty = stderr only).
    std::string file_path;

    /// Maximum log file size before rotation (bytes, 0 = no rotation).
    size_t      max_file_bytes         = 50 * 1024 * 1024;  // 50 MB

    /// Number of rotated files to keep.
    int         max_rotated_files      = 3;

    /// Rate-limit DEBUG messages to once per N scans.
    int         debug_rate_limit       = 10;
};

/// Master daemon configuration.
struct DaemonConfig {
    SensorConfig            sensor;
    odom::SlamEngineConfig  slam;
    IpcConfig               ipc;
    HealthConfig            health;
    CheckpointConfig        checkpoint;
    LogConfig               log;
};

// ═════════════════════════════════════════════════════════════════════════════
//  Shared-memory pose layout (seqlock pattern)
// ═════════════════════════════════════════════════════════════════════════════
//
// The SHM segment has the following layout:
//
//   offset 0:    ShmHeader (64 bytes, cache-line aligned)
//   offset 64:   Pose6DRecord (456 bytes)
//   offset 520:  [padding to page boundary]
//
// The seqlock guarantees tear-free reads without mutexes:
//
//   Writer:                          Reader:
//     seq = seq + 1  (odd → writing)   do {
//     fence(release)                      s1 = seq.load(acquire)
//     memcpy(pose)                        if (s1 & 1) continue  // writer active
//     fence(release)                      fence(acquire)
//     seq = seq + 1  (even → valid)       memcpy(pose)
//                                         fence(acquire)
//                                         s2 = seq.load(acquire)
//                                       } while (s1 != s2)
//
// ─────────────────────────────────────────────────────────────────────────────

#pragma pack(push, 1)

struct ShmHeader {
    uint64_t magic;                 ///< 0x534C414D44504F53 ("SLAMDPOS")
    uint32_t version;               ///< SHM layout version (1)
    uint32_t pose_offset;           ///< byte offset to Pose6DRecord
    uint32_t pose_size;             ///< sizeof(Pose6DRecord)
    uint32_t _reserved;
    uint64_t sequence;              ///< seqlock counter (even = valid)
    int64_t  timestamp_ns;          ///< pose timestamp
    uint8_t  tracking_status;       ///< TrackingStatus enum value
    uint8_t  _pad[7];
};

#pragma pack(pop)

static_assert(sizeof(ShmHeader) == 48, "ShmHeader must be 48 bytes");

inline constexpr uint64_t kShmMagic   = 0x534C414D44504F53ULL;  // "SLAMDPOS"
inline constexpr uint32_t kShmVersion = 1;

// ═════════════════════════════════════════════════════════════════════════════
//  Checkpoint file layout
// ═════════════════════════════════════════════════════════════════════════════

#pragma pack(push, 1)

struct CheckpointHeader {
    char     magic[8];             ///< "SLAMCHKP"
    uint32_t version;              ///< checkpoint format version (1)
    uint32_t payload_size;         ///< bytes after this header
    int64_t  timestamp_ns;         ///< pose timestamp when checkpointed
    uint32_t sequence;             ///< scan counter at checkpoint
    uint32_t crc32;                ///< CRC32 of all bytes after this header
};

struct CheckpointPayload {
    // ESIKF state
    double   quaternion[4];
    double   position[3];
    double   velocity[3];
    double   gyro_bias[3];
    double   accel_bias[3];
    double   gravity[3];

    // Map digest (not full map — map is rebuilt from live data)
    uint64_t map_point_count;
    double   map_center[3];
    double   map_radius;

    // Tracking status
    uint8_t  tracking_status;
    uint8_t  _pad[7];
};

#pragma pack(pop)

static_assert(sizeof(CheckpointPayload) == 208,
              "CheckpointPayload must be 208 bytes");

inline constexpr char kCheckpointMagic[8] = {'S','L','A','M','C','H','K','P'};
inline constexpr uint32_t kCheckpointVersion = 1;

// ═════════════════════════════════════════════════════════════════════════════
//  Health status snapshot
// ═════════════════════════════════════════════════════════════════════════════

struct HealthStatus {
    // ── Process info ────────────────────────────────────────────────────
    double   uptime_s{0};              ///< seconds since daemon start
    double   cpu_usage{0};             ///< 0.0–1.0 (fraction of one core)
    size_t   rss_bytes{0};             ///< resident set size

    // ── SLAM pipeline ───────────────────────────────────────────────────
    odom::TrackingStatus tracking{odom::TrackingStatus::Initializing};
    odom::OdometryStats  odom_stats;

    // ── Time sync ───────────────────────────────────────────────────────
    double   drift_ns_per_sec{0};      ///< clock drift estimate
    bool     drift_warning{false};     ///< true if drift exceeds threshold

    // ── Queue health ────────────────────────────────────────────────────
    size_t   imu_drops{0};
    size_t   cloud_drops{0};

    // ── Sensor status ───────────────────────────────────────────────────
    bool     sensor_connected{false};
    bool     sensor_streaming{false};

    // ── Checkpoint ──────────────────────────────────────────────────────
    int64_t  last_checkpoint_ns{0};    ///< timestamp of last checkpoint
    bool     warm_started{false};      ///< true if restored from checkpoint
};

// ═════════════════════════════════════════════════════════════════════════════
//  SlamDaemon — top-level process orchestrator
// ═════════════════════════════════════════════════════════════════════════════

class SlamDaemon {
public:
    explicit SlamDaemon(DaemonConfig config);
    ~SlamDaemon();

    // Non-copyable
    SlamDaemon(const SlamDaemon&) = delete;
    SlamDaemon& operator=(const SlamDaemon&) = delete;

    // ── Lifecycle ───────────────────────────────────────────────────────

    /// Initialise all subsystems.  Returns false on fatal error.
    bool start();

    /// Block until shutdown signal is received.
    /// Call from main() after start().
    void run();

    /// Signal graceful shutdown (can be called from signal handler).
    void requestShutdown();

    /// Clean up all subsystems.
    void stop();

    // ── Runtime control ─────────────────────────────────────────────────

    /// Reload configuration (SIGHUP handler).
    /// Currently reloads log level and health thresholds without restart.
    bool reloadConfig(const std::string& config_path);

    /// Reset the SLAM engine (re-initialise estimator, keep config).
    void resetSlam();

    // ── Status ──────────────────────────────────────────────────────────

    /// Current health snapshot.
    [[nodiscard]] HealthStatus health() const;

    /// True if the daemon is running.
    [[nodiscard]] bool isRunning() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// ═════════════════════════════════════════════════════════════════════════════
//  Config loader
// ═════════════════════════════════════════════════════════════════════════════

/// Load a DaemonConfig from a YAML file.
/// Returns false and fills `error` on parse failure.
bool loadConfig(const std::string& path, DaemonConfig& config,
                std::string& error);

} // namespace thunderbird::slamd
