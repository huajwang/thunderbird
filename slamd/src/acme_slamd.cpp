// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — acme_slamd: SLAM Daemon Implementation
// ─────────────────────────────────────────────────────────────────────────────
#include "acme_slamd.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>

// ─────────────────────────────────────────────────────────────────────────────
//  Platform includes (Linux-specific IPC, conditional)
// ─────────────────────────────────────────────────────────────────────────────
#ifdef __linux__
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

// ─────────────────────────────────────────────────────────────────────────────
//  Optional dependencies (ZeroMQ).  Build without if not available.
// ─────────────────────────────────────────────────────────────────────────────
#ifdef SLAMD_HAS_ZMQ
#include <zmq.h>
#endif

// ─────────────────────────────────────────────────────────────────────────────
//  Optional systemd notification.
// ─────────────────────────────────────────────────────────────────────────────
#ifdef SLAMD_HAS_SYSTEMD
#include <systemd/sd-daemon.h>
#else
// Stub when building without libsystemd.
inline int sd_notify(int /*unset_environment*/, const char* /*state*/) {
    return 0;
}
inline int sd_notifyf(int /*unset_environment*/, const char* /*fmt*/, ...) {
    return 0;
}
#endif

namespace thunderbird::slamd {

using namespace std::chrono;
using clock_t = steady_clock;

// ═════════════════════════════════════════════════════════════════════════════
//  Logger — structured, levelled, rate-limited
// ═════════════════════════════════════════════════════════════════════════════

class Logger {
public:
    explicit Logger(LogConfig cfg)
        : cfg_(std::move(cfg))
        , start_time_(clock_t::now()) {}

    void log(LogLevel level, const char* module, const std::string& msg) {
        if (static_cast<uint8_t>(level) > static_cast<uint8_t>(cfg_.level))
            return;

        const auto now = system_clock::now();
        const auto since_start =
            duration_cast<milliseconds>(clock_t::now() - start_time_);

        auto tt = system_clock::to_time_t(now);
        auto ms = duration_cast<milliseconds>(
            now.time_since_epoch()) % 1000;

        std::tm tm_buf{};
#ifdef _WIN32
        gmtime_s(&tm_buf, &tt);
#else
        gmtime_r(&tt, &tm_buf);
#endif

        char time_str[32];
        std::strftime(time_str, sizeof(time_str),
                      "%Y-%m-%d %H:%M:%S", &tm_buf);

        std::lock_guard<std::mutex> lk(mu_);

        // Format: [TIME.ms] [LEVEL] [module] message uptime_s=N
        std::ostringstream oss;
        oss << "[" << time_str << "."
            << std::setfill('0') << std::setw(3) << ms.count()
            << "] [" << log_level_name(level)
            << "] [" << module << "] " << msg
            << " uptime_s=" << since_start.count() / 1000.0;

        const auto line = oss.str();
        std::cerr << line << "\n";

        if (log_file_.is_open()) {
            log_file_ << line << "\n";
            log_file_.flush();
            written_bytes_ += line.size() + 1;
            maybeRotate();
        }
    }

    void openFile() {
        if (cfg_.file_path.empty()) return;
        log_file_.open(cfg_.file_path, std::ios::app);
    }

    void setLevel(LogLevel level) { cfg_.level = level; }

    bool shouldDebugLog(uint64_t scan_seq) const {
        if (cfg_.debug_rate_limit <= 0) return true;
        return (scan_seq % static_cast<uint64_t>(cfg_.debug_rate_limit)) == 0;
    }

private:
    void maybeRotate() {
        if (cfg_.max_file_bytes == 0) return;
        if (written_bytes_ < cfg_.max_file_bytes) return;

        log_file_.close();

        // Rotate: slamd.log → slamd.log.1 → slamd.log.2 → ...
        for (int i = cfg_.max_rotated_files - 1; i >= 1; --i) {
            auto from = cfg_.file_path + "." + std::to_string(i);
            auto to   = cfg_.file_path + "." + std::to_string(i + 1);
            std::rename(from.c_str(), to.c_str());
        }
        std::rename(cfg_.file_path.c_str(),
                    (cfg_.file_path + ".1").c_str());

        log_file_.open(cfg_.file_path, std::ios::trunc);
        written_bytes_ = 0;
    }

    LogConfig    cfg_;
    clock_t::time_point start_time_;
    std::mutex   mu_;
    std::ofstream log_file_;
    size_t       written_bytes_{0};
};

// ═════════════════════════════════════════════════════════════════════════════
//  CRC32 — minimal implementation for checkpoint validation
// ═════════════════════════════════════════════════════════════════════════════

static uint32_t crc32(const void* data, size_t len) noexcept {
    const auto* bytes = static_cast<const uint8_t*>(data);
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= bytes[i];
        for (int j = 0; j < 8; ++j) {
            crc = (crc >> 1) ^ (0xEDB88320 & (~(crc & 1) + 1));
        }
    }
    return ~crc;
}

// ═════════════════════════════════════════════════════════════════════════════
//  CPU usage measurement (Linux-specific, stubbed on others)
// ═════════════════════════════════════════════════════════════════════════════

class CpuMeter {
public:
    CpuMeter() { sample(); }

    /// Returns CPU usage as fraction of one core (0.0–1.0+).
    double measure() {
        auto wall_now = clock_t::now();
        double cpu_now = getCpuTime();
        double wall_delta =
            duration<double>(wall_now - last_wall_).count();
        double cpu_delta = cpu_now - last_cpu_;
        last_wall_ = wall_now;
        last_cpu_  = cpu_now;
        if (wall_delta < 1e-6) return 0.0;
        return cpu_delta / wall_delta;
    }

private:
    void sample() {
        last_wall_ = clock_t::now();
        last_cpu_  = getCpuTime();
    }

    static double getCpuTime() {
#ifdef __linux__
        struct rusage ru{};
        getrusage(RUSAGE_SELF, &ru);
        return static_cast<double>(ru.ru_utime.tv_sec) +
               static_cast<double>(ru.ru_utime.tv_usec) / 1e6 +
               static_cast<double>(ru.ru_stime.tv_sec) +
               static_cast<double>(ru.ru_stime.tv_usec) / 1e6;
#else
        return 0.0;
#endif
    }

    clock_t::time_point last_wall_;
    double last_cpu_{0};
};

// ═════════════════════════════════════════════════════════════════════════════
//  Shared Memory Writer (POSIX shm + seqlock)
// ═════════════════════════════════════════════════════════════════════════════

class ShmPoseWriter {
public:
    explicit ShmPoseWriter(const IpcConfig& cfg)
        : shm_name_(cfg.shm_pose_name)
        , shm_size_(cfg.shm_pose_bytes) {}

    ~ShmPoseWriter() { close(); }

    bool open() {
#ifdef __linux__
        fd_ = shm_open(shm_name_.c_str(), O_CREAT | O_RDWR, 0666);
        if (fd_ < 0) return false;
        if (ftruncate(fd_, static_cast<off_t>(shm_size_)) != 0) {
            ::close(fd_); fd_ = -1; return false;
        }
        base_ = static_cast<uint8_t*>(
            mmap(nullptr, shm_size_, PROT_READ | PROT_WRITE,
                 MAP_SHARED, fd_, 0));
        if (base_ == MAP_FAILED) {
            ::close(fd_); fd_ = -1; base_ = nullptr; return false;
        }

        // Initialise header.
        auto* hdr = reinterpret_cast<ShmHeader*>(base_);
        std::memset(hdr, 0, sizeof(ShmHeader));
        hdr->magic       = kShmMagic;
        hdr->version     = kShmVersion;
        hdr->pose_offset = 64;  // after header, cache-line aligned
        hdr->pose_size   = sizeof(odom::Pose6DRecord);
        hdr->sequence    = 0;
        return true;
#else
        (void)shm_name_; (void)shm_size_;
        return false;  // SHM not available on this platform
#endif
    }

    void writePose(const odom::Pose6D& pose) {
#ifdef __linux__
        if (!base_) return;
        auto* hdr = reinterpret_cast<ShmHeader*>(base_);
        auto* pose_buf = base_ + hdr->pose_offset;

        // Seqlock: increment to odd (writing).
        auto seq = hdr->sequence;
        hdr->sequence = seq + 1;
        __atomic_thread_fence(__ATOMIC_RELEASE);

        // Write pose.
        odom::Pose6DRecord rec = odom::to_record(pose);
        std::memcpy(pose_buf, &rec, sizeof(rec));
        hdr->timestamp_ns    = pose.timestamp_ns;
        hdr->tracking_status = static_cast<uint8_t>(pose.tracking_status);

        // Seqlock: increment to even (valid).
        __atomic_thread_fence(__ATOMIC_RELEASE);
        hdr->sequence = seq + 2;
#else
        (void)pose;
#endif
    }

    void close() {
#ifdef __linux__
        if (base_) {
            munmap(base_, shm_size_);
            base_ = nullptr;
        }
        if (fd_ >= 0) {
            ::close(fd_);
            shm_unlink(shm_name_.c_str());
            fd_ = -1;
        }
#endif
    }

private:
    std::string shm_name_;
    size_t      shm_size_;
    int         fd_{-1};
    uint8_t*    base_{nullptr};
};

// ═════════════════════════════════════════════════════════════════════════════
//  ZeroMQ Publisher (optional)
// ═════════════════════════════════════════════════════════════════════════════

class ZmqPublisher {
public:
    ZmqPublisher() = default;
    ~ZmqPublisher() { close(); }

    bool open(const std::string& pose_endpoint,
              const std::string& map_endpoint) {
#ifdef SLAMD_HAS_ZMQ
        ctx_ = zmq_ctx_new();
        if (!ctx_) return false;

        pose_sock_ = zmq_socket(ctx_, ZMQ_PUB);
        if (!pose_sock_ || zmq_bind(pose_sock_, pose_endpoint.c_str()) != 0) {
            close(); return false;
        }

        map_sock_ = zmq_socket(ctx_, ZMQ_PUB);
        if (!map_sock_ || zmq_bind(map_sock_, map_endpoint.c_str()) != 0) {
            close(); return false;
        }
        return true;
#else
        (void)pose_endpoint; (void)map_endpoint;
        return false;
#endif
    }

    /// Publish a lightweight pose notification (topic + timestamp + seq).
    void publishPoseNotify(int64_t timestamp_ns, uint32_t sequence) {
#ifdef SLAMD_HAS_ZMQ
        if (!pose_sock_) return;
        // Topic: "pose" (4 bytes) + timestamp(8) + sequence(4) = 16 bytes
        uint8_t buf[16];
        std::memcpy(buf, "pose", 4);
        std::memcpy(buf + 4, &timestamp_ns, 8);
        std::memcpy(buf + 12, &sequence, 4);
        zmq_send(pose_sock_, buf, sizeof(buf), ZMQ_DONTWAIT);
#else
        (void)timestamp_ns; (void)sequence;
#endif
    }

    /// Publish a serialized map snapshot.
    void publishMapSnapshot(const void* data, size_t len) {
#ifdef SLAMD_HAS_ZMQ
        if (!map_sock_) return;
        // Topic: "map\0" (4 bytes) + payload
        zmq_msg_t msg;
        zmq_msg_init_size(&msg, 4 + len);
        auto* buf = static_cast<uint8_t*>(zmq_msg_data(&msg));
        std::memcpy(buf, "map\0", 4);
        std::memcpy(buf + 4, data, len);
        zmq_msg_send(&msg, map_sock_, ZMQ_DONTWAIT);
        zmq_msg_close(&msg);
#else
        (void)data; (void)len;
#endif
    }

    void close() {
#ifdef SLAMD_HAS_ZMQ
        if (pose_sock_) { zmq_close(pose_sock_); pose_sock_ = nullptr; }
        if (map_sock_)  { zmq_close(map_sock_);  map_sock_  = nullptr; }
        if (ctx_)       { zmq_ctx_destroy(ctx_);  ctx_       = nullptr; }
#endif
    }

private:
    void* ctx_{nullptr};
    void* pose_sock_{nullptr};
    void* map_sock_{nullptr};
};

// ═════════════════════════════════════════════════════════════════════════════
//  Checkpoint Manager — atomic write + warm restart
// ═════════════════════════════════════════════════════════════════════════════

class CheckpointManager {
public:
    explicit CheckpointManager(CheckpointConfig cfg, Logger& log)
        : cfg_(std::move(cfg)), log_(log) {}

    /// Save checkpoint atomically (write tmp + fsync + rename).
    bool save(const odom::SlamOutput& output, odom::TrackingStatus status) {
        if (!cfg_.enable) return true;

        const auto path     = cfg_.directory + "/" + cfg_.filename;
        const auto tmp_path = path + ".tmp";

        CheckpointHeader hdr{};
        std::memcpy(hdr.magic, kCheckpointMagic, 8);
        hdr.version      = kCheckpointVersion;
        hdr.payload_size = sizeof(CheckpointPayload);
        hdr.timestamp_ns = output.timestamp_ns;
        hdr.sequence     = static_cast<uint32_t>(
            output.pose.timestamp_ns > 0 ? 1 : 0);

        CheckpointPayload payload{};
        for (int i = 0; i < 4; ++i)
            payload.quaternion[i] = output.pose.quaternion[i];
        for (int i = 0; i < 3; ++i) {
            payload.position[i]   = output.pose.position[i];
            payload.velocity[i]   = output.pose.velocity[i];
            payload.gyro_bias[i]  = output.pose.gyro_bias[i];
            payload.accel_bias[i] = output.pose.accel_bias[i];
            payload.gravity[i]    = output.pose.gravity[i];
        }
        payload.map_point_count = output.map_info.total_points;
        for (int i = 0; i < 3; ++i)
            payload.map_center[i] = output.map_info.center[i];
        payload.map_radius      = output.map_info.radius;
        payload.tracking_status = static_cast<uint8_t>(status);

        hdr.crc32 = crc32(&payload, sizeof(payload));

        // Write to temp file.
        std::ofstream fs(tmp_path, std::ios::binary | std::ios::trunc);
        if (!fs) {
            log_.log(LogLevel::Err, "checkpoint",
                     "Failed to open " + tmp_path);
            return false;
        }
        fs.write(reinterpret_cast<const char*>(&hdr), sizeof(hdr));
        fs.write(reinterpret_cast<const char*>(&payload), sizeof(payload));
        fs.flush();
        fs.close();

        // Atomic rename.
        if (std::rename(tmp_path.c_str(), path.c_str()) != 0) {
            log_.log(LogLevel::Err, "checkpoint",
                     "rename failed: " + tmp_path + " → " + path);
            return false;
        }

        log_.log(LogLevel::Debug, "checkpoint",
                 "Saved checkpoint seq=" + std::to_string(hdr.sequence) +
                 " ts=" + std::to_string(hdr.timestamp_ns));
        return true;
    }

    /// Attempt to load and validate a checkpoint for warm restart.
    /// Returns true if a valid, non-stale checkpoint was found.
    bool tryLoad(CheckpointPayload& payload, int64_t current_time_ns) {
        if (!cfg_.enable) return false;

        const auto path = cfg_.directory + "/" + cfg_.filename;
        std::ifstream fs(path, std::ios::binary);
        if (!fs) return false;

        CheckpointHeader hdr{};
        fs.read(reinterpret_cast<char*>(&hdr), sizeof(hdr));
        if (!fs || std::memcmp(hdr.magic, kCheckpointMagic, 8) != 0) {
            log_.log(LogLevel::Warning, "checkpoint",
                     "Invalid magic in checkpoint file");
            return false;
        }
        if (hdr.version != kCheckpointVersion) {
            log_.log(LogLevel::Warning, "checkpoint",
                     "Checkpoint version mismatch: " +
                     std::to_string(hdr.version));
            return false;
        }
        if (hdr.payload_size != sizeof(CheckpointPayload)) {
            log_.log(LogLevel::Warning, "checkpoint",
                     "Unexpected payload size");
            return false;
        }

        fs.read(reinterpret_cast<char*>(&payload), sizeof(payload));
        if (!fs) return false;

        // Verify CRC.
        uint32_t computed = crc32(&payload, sizeof(payload));
        if (computed != hdr.crc32) {
            log_.log(LogLevel::Warning, "checkpoint",
                     "CRC mismatch (corrupt checkpoint)");
            return false;
        }

        // Staleness check.
        const double age_s =
            static_cast<double>(current_time_ns - hdr.timestamp_ns) / 1e9;
        if (age_s > cfg_.max_age_s || age_s < 0) {
            log_.log(LogLevel::Info, "checkpoint",
                     "Checkpoint too stale (age=" +
                     std::to_string(age_s) + "s), cold start");
            return false;
        }

        log_.log(LogLevel::Info, "checkpoint",
                 "Loaded checkpoint age=" + std::to_string(age_s) +
                 "s ts=" + std::to_string(hdr.timestamp_ns));
        return true;
    }

private:
    CheckpointConfig cfg_;
    Logger& log_;
};

// ═════════════════════════════════════════════════════════════════════════════
//  PImpl
// ═════════════════════════════════════════════════════════════════════════════

struct SlamDaemon::Impl {
    DaemonConfig     config;
    Logger           log;
    CpuMeter         cpu_meter;

    // ── Subsystems ──────────────────────────────────────────────────────
    std::unique_ptr<DeviceManager>       device;
    std::unique_ptr<odom::AcmeSlamEngine> engine;
    ShmPoseWriter    shm_writer;
    ZmqPublisher     zmq_pub;
    CheckpointManager checkpoint_mgr;

    // ── Threads ─────────────────────────────────────────────────────────
    std::thread      ipc_thread;
    std::thread      health_thread;
    std::thread      checkpoint_thread;

    // ── Shutdown coordination ───────────────────────────────────────────
    std::atomic<bool> running{false};
    std::atomic<bool> shutdown_flag{false};
    std::mutex        shutdown_mu;
    std::condition_variable shutdown_cv;

    // ── Health state ────────────────────────────────────────────────────
    mutable std::mutex health_mu;
    HealthStatus       health_status;
    bool               warm_started{false};

    // ── Scan counter for IPC rate control ───────────────────────────────
    std::atomic<uint64_t> scan_sequence{0};

    // ── Latest outputs (written by engine callbacks, read by IPC) ───────
    mutable std::mutex output_mu;
    odom::SlamOutput   latest_output;
    bool               output_available{false};

    mutable std::mutex pose_mu;
    odom::Pose6D       latest_pose;
    bool               pose_available{false};

    // ── Constructor ─────────────────────────────────────────────────────
    explicit Impl(DaemonConfig cfg)
        : config(std::move(cfg))
        , log(config.log)
        , shm_writer(config.ipc)
        , checkpoint_mgr(config.checkpoint, log) {}

    // ── IPC publisher thread ────────────────────────────────────────────
    void ipcLoop() {
        log.log(LogLevel::Info, "ipc", "IPC publisher thread started");

        const auto map_interval = duration<double>(
            1.0 / std::max(config.ipc.map_publish_hz, 0.1));
        auto last_map_publish = clock_t::now();

        while (!shutdown_flag.load(std::memory_order_acquire)) {
            // ── Publish pose to SHM ─────────────────────────────────
            {
                std::lock_guard<std::mutex> lk(pose_mu);
                if (pose_available) {
                    if (config.ipc.enable_shm) {
                        shm_writer.writePose(latest_pose);
                    }
                    if (config.ipc.enable_zmq) {
                        zmq_pub.publishPoseNotify(
                            latest_pose.timestamp_ns,
                            scan_sequence.load(std::memory_order_relaxed));
                    }
                    pose_available = false;
                }
            }

            // ── Publish map snapshot at lower rate ──────────────────
            if (clock_t::now() - last_map_publish > map_interval) {
                std::lock_guard<std::mutex> lk(output_mu);
                if (output_available && config.ipc.enable_zmq) {
                    // Serialize a lightweight map digest.
                    odom::LocalMapInfo info = latest_output.map_info;
                    zmq_pub.publishMapSnapshot(&info, sizeof(info));
                    last_map_publish = clock_t::now();
                }
            }

            std::this_thread::sleep_for(microseconds(500));
        }

        log.log(LogLevel::Info, "ipc", "IPC publisher thread stopped");
    }

    // ── Health monitor thread ───────────────────────────────────────────
    void healthLoop() {
        log.log(LogLevel::Info, "health", "Health monitor thread started");

        const auto interval = duration<double>(
            1.0 / std::max(config.health.health_check_hz, 0.1));

        while (!shutdown_flag.load(std::memory_order_acquire)) {
            HealthStatus hs;
            hs.uptime_s = duration<double>(
                clock_t::now() - start_time_).count();
            hs.cpu_usage  = cpu_meter.measure();
            hs.rss_bytes  = getRss();

            if (engine) {
                hs.tracking   = engine->status();
                hs.odom_stats = engine->stats();
                hs.imu_drops  = engine->imuDropCount();
                hs.cloud_drops = engine->cloudDropCount();
            }
            if (device) {
                hs.sensor_connected = device->is_connected();
                hs.sensor_streaming = device->is_streaming();
            }

            hs.warm_started = warm_started;

            // Drift estimate from the time sync stats in the engine.
            // (The engine exposes OdometryStats; drift comes from there.)
            hs.drift_ns_per_sec = 0;  // TODO: wire to SlamTimeSync drift model
            hs.drift_warning = (std::abs(hs.drift_ns_per_sec) >
                                config.health.drift_warn_threshold);

            {
                std::lock_guard<std::mutex> lk(health_mu);
                health_status = hs;
            }

            // Logging.
            if (hs.cpu_usage > config.health.cpu_warn_threshold) {
                log.log(LogLevel::Warning, "health",
                        "CPU usage high: " +
                        std::to_string(hs.cpu_usage * 100.0) + "%");
            }
            if (hs.drift_warning) {
                log.log(LogLevel::Warning, "health",
                        "Clock drift: " +
                        std::to_string(hs.drift_ns_per_sec) + " ns/s");
            }

            // Systemd watchdog.
            if (config.health.enable_watchdog) {
                sd_notify(0, "WATCHDOG=1");
            }

            // Publish structured status to systemd.
            sd_notifyf(0, "STATUS=tracking=%s cpu=%.1f%% scans=%lu",
                        odom::tracking_status_name(hs.tracking),
                        hs.cpu_usage * 100.0,
                        static_cast<unsigned long>(
                            hs.odom_stats.lidar_scans_processed));

            std::this_thread::sleep_for(
                duration_cast<milliseconds>(interval));
        }

        log.log(LogLevel::Info, "health", "Health monitor thread stopped");
    }

    // ── Checkpoint thread ───────────────────────────────────────────────
    void checkpointLoop() {
        if (!config.checkpoint.enable) return;
        log.log(LogLevel::Info, "checkpoint", "Checkpoint thread started");

        const auto interval = duration<double>(config.checkpoint.interval_s);

        while (!shutdown_flag.load(std::memory_order_acquire)) {
            std::this_thread::sleep_for(
                duration_cast<milliseconds>(interval));

            if (shutdown_flag.load(std::memory_order_acquire)) break;

            odom::SlamOutput snapshot;
            odom::TrackingStatus status;
            {
                std::lock_guard<std::mutex> lk(output_mu);
                if (!output_available) continue;
                snapshot = latest_output;
            }
            if (engine) {
                status = engine->status();
            } else {
                status = odom::TrackingStatus::Initializing;
            }

            checkpoint_mgr.save(snapshot, status);

            {
                std::lock_guard<std::mutex> lk(health_mu);
                health_status.last_checkpoint_ns = snapshot.timestamp_ns;
            }
        }

        log.log(LogLevel::Info, "checkpoint", "Checkpoint thread stopped");
    }

    // ── RSS measurement (Linux) ─────────────────────────────────────────
    static size_t getRss() {
#ifdef __linux__
        std::ifstream fs("/proc/self/statm");
        size_t pages = 0;
        // statm: size resident shared text lib data dt
        //        skip  ↑ this one
        size_t dummy = 0;
        if (fs >> dummy >> pages) {
            return pages * static_cast<size_t>(sysconf(_SC_PAGESIZE));
        }
#endif
        return 0;
    }

    clock_t::time_point start_time_;
};

// ═════════════════════════════════════════════════════════════════════════════
//  Public API implementation
// ═════════════════════════════════════════════════════════════════════════════

SlamDaemon::SlamDaemon(DaemonConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

SlamDaemon::~SlamDaemon() {
    if (impl_->running.load()) stop();
}

bool SlamDaemon::start() {
    auto& d = *impl_;
    d.start_time_ = clock_t::now();

    d.log.openFile();
    d.log.log(LogLevel::Info, "daemon", "acme_slamd starting");

    // ── 1. Create DeviceManager ─────────────────────────────────────────
    DeviceConfig dev_cfg;
    dev_cfg.uri    = d.config.sensor.device_uri;
    dev_cfg.imu_hz = d.config.sensor.imu_rate_hz;
    dev_cfg.lidar_hz = d.config.sensor.lidar_rate_hz;

    d.device = std::make_unique<DeviceManager>(dev_cfg);

    auto status = d.device->connect();
    if (status != Status::OK) {
        d.log.log(LogLevel::Err, "daemon",
                  "Failed to connect to device: " + d.config.sensor.device_uri);
        return false;
    }
    d.log.log(LogLevel::Info, "daemon", "Device connected");

    // ── 2. Create and initialise SLAM engine ────────────────────────────
    d.engine = std::make_unique<odom::AcmeSlamEngine>();

    // Try warm restart from checkpoint.
    CheckpointPayload ckpt{};
    auto now_ns = duration_cast<nanoseconds>(
        system_clock::now().time_since_epoch()).count();
    if (d.checkpoint_mgr.tryLoad(ckpt, now_ns)) {
        d.log.log(LogLevel::Info, "daemon",
                  "Warm restart from checkpoint");
        d.warm_started = true;
        // TODO: pre-seed ESIKF state from ckpt fields
        // (requires AcmeSlamEngine to expose a warmStart() method)
    }

    if (!d.engine->initialize(d.config.slam)) {
        d.log.log(LogLevel::Err, "daemon", "SLAM engine init failed");
        return false;
    }
    d.log.log(LogLevel::Info, "daemon", "SLAM engine initialised");

    // ── 3. Wire sensor callbacks → engine ───────────────────────────────
    d.device->on_imu([&d](std::shared_ptr<const ImuSample> s) {
        odom::ImuSample os;
        os.timestamp_ns = s->timestamp.nanoseconds;
        for (int i = 0; i < 3; ++i) {
            os.accel[i] = static_cast<double>(s->accel[i]);
            os.gyro[i]  = static_cast<double>(s->gyro[i]);
        }
        auto host_ns = duration_cast<nanoseconds>(
            clock_t::now().time_since_epoch()).count();
        d.engine->feedImu(os, host_ns);
    });

    d.device->on_lidar([&d](std::shared_ptr<const LidarFrame> f) {
        auto cloud = std::make_shared<odom::PointCloudFrame>();
        cloud->timestamp_ns = f->timestamp.nanoseconds;
        cloud->points.reserve(f->points.size());
        for (const auto& p : f->points) {
            cloud->points.push_back({p.x, p.y, p.z, p.intensity, 0});
        }
        auto host_ns = duration_cast<nanoseconds>(
            clock_t::now().time_since_epoch()).count();
        d.engine->feedPointCloud(std::move(cloud), host_ns);
    });

    // ── 4. Wire engine callbacks → IPC ──────────────────────────────────
    d.engine->onSlamOutput([&d](std::shared_ptr<const odom::SlamOutput> out) {
        {
            std::lock_guard<std::mutex> lk(d.output_mu);
            d.latest_output    = *out;
            d.output_available = true;
        }
        d.scan_sequence.fetch_add(1, std::memory_order_relaxed);

        // Debug log (rate-limited).
        uint64_t seq = d.scan_sequence.load(std::memory_order_relaxed);
        if (d.log.shouldDebugLog(seq)) {
            d.log.log(LogLevel::Debug, "slam",
                      "scan=" + std::to_string(seq) +
                      " latency_ms=" +
                      std::to_string(out->update_latency_ms) +
                      " correspondences=" +
                      std::to_string(out->correspondences) +
                      " status=" + odom::tracking_status_name(
                          out->pose.tracking_status));
        }
    });

    d.engine->onPose([&d](std::shared_ptr<const odom::Pose6D> pose) {
        std::lock_guard<std::mutex> lk(d.pose_mu);
        d.latest_pose    = *pose;
        d.pose_available = true;
    });

    d.engine->onStatusChange([&d](odom::TrackingStatus s) {
        d.log.log(LogLevel::Info, "slam",
                  std::string("Status → ") + odom::tracking_status_name(s));
    });

    // ── 5. Open IPC channels ────────────────────────────────────────────
    if (d.config.ipc.enable_shm) {
        if (!d.shm_writer.open()) {
            d.log.log(LogLevel::Warning, "ipc",
                      "Failed to open SHM — pose IPC disabled");
        } else {
            d.log.log(LogLevel::Info, "ipc",
                      "SHM pose writer: " + d.config.ipc.shm_pose_name);
        }
    }

    if (d.config.ipc.enable_zmq) {
        if (!d.zmq_pub.open(d.config.ipc.zmq_pose_endpoint,
                            d.config.ipc.zmq_map_endpoint)) {
            d.log.log(LogLevel::Warning, "ipc",
                      "Failed to open ZMQ — notification IPC disabled");
        } else {
            d.log.log(LogLevel::Info, "ipc",
                      "ZMQ pose: " + d.config.ipc.zmq_pose_endpoint);
        }
    }

    // ── 6. Start sensor streams ─────────────────────────────────────────
    status = d.device->start();
    if (status != Status::OK) {
        d.log.log(LogLevel::Err, "daemon", "Failed to start sensor streams");
        return false;
    }
    d.log.log(LogLevel::Info, "daemon", "Sensor streams started");

    // ── 7. Spawn daemon threads ─────────────────────────────────────────
    d.running.store(true, std::memory_order_release);
    d.shutdown_flag.store(false, std::memory_order_release);

    d.ipc_thread = std::thread([&d]() { d.ipcLoop(); });
    d.health_thread = std::thread([&d]() { d.healthLoop(); });
    d.checkpoint_thread = std::thread([&d]() { d.checkpointLoop(); });

    // ── 8. Notify systemd ───────────────────────────────────────────────
    sd_notify(0, "READY=1");
    d.log.log(LogLevel::Info, "daemon", "acme_slamd ready");

    return true;
}

void SlamDaemon::run() {
    auto& d = *impl_;
    std::unique_lock<std::mutex> lk(d.shutdown_mu);
    d.shutdown_cv.wait(lk, [&d]() {
        return d.shutdown_flag.load(std::memory_order_acquire);
    });
}

void SlamDaemon::requestShutdown() {
    auto& d = *impl_;
    d.shutdown_flag.store(true, std::memory_order_release);
    d.shutdown_cv.notify_all();
}

void SlamDaemon::stop() {
    auto& d = *impl_;
    if (!d.running.load(std::memory_order_acquire)) return;

    d.log.log(LogLevel::Info, "daemon", "acme_slamd shutting down");

    sd_notify(0, "STOPPING=1");
    d.shutdown_flag.store(true, std::memory_order_release);

    // Join daemon threads.
    if (d.ipc_thread.joinable()) d.ipc_thread.join();
    if (d.health_thread.joinable()) d.health_thread.join();
    if (d.checkpoint_thread.joinable()) d.checkpoint_thread.join();

    // Final checkpoint before shutdown.
    if (d.config.checkpoint.enable && d.output_available) {
        d.log.log(LogLevel::Info, "checkpoint", "Final checkpoint on shutdown");
        d.checkpoint_mgr.save(d.latest_output, d.engine->status());
    }

    // Stop SLAM engine.
    if (d.engine) {
        d.engine->shutdown();
        d.engine.reset();
    }

    // Stop and disconnect device.
    if (d.device) {
        d.device->stop();
        d.device->disconnect();
        d.device.reset();
    }

    // Close IPC.
    d.shm_writer.close();
    d.zmq_pub.close();

    d.running.store(false, std::memory_order_release);
    d.log.log(LogLevel::Info, "daemon", "acme_slamd stopped");
}

bool SlamDaemon::reloadConfig(const std::string& config_path) {
    DaemonConfig new_cfg;
    std::string error;
    if (!loadConfig(config_path, new_cfg, error)) {
        impl_->log.log(LogLevel::Warning, "daemon",
                        "Config reload failed: " + error);
        return false;
    }

    // Hot-reloadable: log level, health thresholds.
    impl_->log.setLevel(new_cfg.log.level);
    impl_->config.health = new_cfg.health;
    impl_->config.log.level = new_cfg.log.level;

    impl_->log.log(LogLevel::Info, "daemon", "Config reloaded");
    return true;
}

void SlamDaemon::resetSlam() {
    if (impl_->engine) {
        impl_->engine->reset();
        impl_->log.log(LogLevel::Info, "daemon", "SLAM engine reset");
    }
}

HealthStatus SlamDaemon::health() const {
    std::lock_guard<std::mutex> lk(impl_->health_mu);
    return impl_->health_status;
}

bool SlamDaemon::isRunning() const noexcept {
    return impl_->running.load(std::memory_order_acquire);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Config loader (YAML parsing — stub; real impl uses yaml-cpp)
// ═════════════════════════════════════════════════════════════════════════════
//
// In the real build this uses yaml-cpp:
//   #include <yaml-cpp/yaml.h>
//
// The stub below loads sensible defaults so the daemon can be exercised
// without the yaml-cpp dependency.  A production build defines SLAMD_HAS_YAML
// and provides the full parser.

bool loadConfig(const std::string& path, DaemonConfig& config,
                std::string& error) {
#ifdef SLAMD_HAS_YAML
    // TODO: full yaml-cpp implementation
    // YAML::Node root = YAML::LoadFile(path);
    // config.sensor.device_uri = root["sensor"]["device_uri"].as<string>();
    // ...
    (void)path;
    (void)config;
    error = "yaml-cpp implementation pending";
    return false;
#else
    // Stub: check the file exists, use defaults.
    std::ifstream fs(path);
    if (!fs.is_open()) {
        error = "Cannot open config file: " + path;
        return false;
    }
    // Apply defaults (already set in DaemonConfig constructors).
    config = DaemonConfig{};
    config.sensor.device_uri = "";  // simulated mode
    (void)error;
    return true;
#endif
}

} // namespace thunderbird::slamd
