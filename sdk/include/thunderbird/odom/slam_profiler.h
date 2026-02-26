// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Performance Profiler
// ─────────────────────────────────────────────────────────────────────────────
//
// Zero-overhead (when disabled) performance instrumentation for the SLAM
// pipeline.  Provides:
//
//   1. Scoped timing macros (compile-time gated).
//   2. Lock-free per-thread histogram accumulators.
//   3. Global profiling registry with string-keyed probes.
//   4. Memory / CPU / drift metrics.
//   5. Report generation (JSON, CSV, human-readable).
//
// ═══════════════════════════════════════════════════════════════════════════
//  Instrumentation Strategy
// ═══════════════════════════════════════════════════════════════════════════
//
//  Compile-time gate:
//    Define THUNDERBIRD_SLAM_PROFILING=1 to enable all instrumentation.
//    When disabled (default), every macro expands to nothing — zero cost.
//
//  Runtime gate:
//    Even when compiled in, individual probes can be armed/disarmed via
//    SlamProfiler::enable(name) / SlamProfiler::disable(name).
//
//  Probe placement (hot-path budget per probe: < 25 ns):
//
//    ┌─────────────────────────────────────────────────────────────────┐
//    │  Probe Name              Location              Expected Hz     │
//    ├─────────────────────────────────────────────────────────────────┤
//    │  imu_propagate           EsikfEngine::propagate     200–1000  │
//    │  scan_deskew             PointDeskewer::deskew       10        │
//    │  esikf_update            EsikfEngine::update         10        │
//    │  ikd_insert              IkdTreeWrapper::insert      10        │
//    │  ikd_search              nearest-neighbor search     10        │
//    │  time_sync_assemble      SlamTimeSync::poll_next     10        │
//    │  output_publish          worker publish path         10        │
//    │  end_to_end              feed→output latency         10        │
//    │  checkpoint_write        CheckpointManager           0.2       │
//    │  shm_write               ShmPoseWriter              200        │
//    └─────────────────────────────────────────────────────────────────┘
//
//  Data flow:
//
//    SLAM_PROFILE_SCOPE("esikf_update")
//           │
//           ▼
//    ScopedTimer (rdtsc or steady_clock)
//           │ dtor records elapsed
//           ▼
//    ProbeAccumulator  (per-probe, cache-line padded)
//        .count++, .sum += dt, min/max, histogram[bucket]++
//           │
//           ▼
//    SlamProfiler::snapshot()  →  ProfileSnapshot (value type, copyable)
//           │
//           ▼
//    report_json() / report_csv() / report_text()
//
//  Memory overhead:
//    Each probe: 256 bytes (padded to cache line).
//    Default 16 probes = 4 KB total.
//    Histogram: 20 log₂-spaced buckets per probe (in-struct).
//
//  Thread safety:
//    Each probe accumulator is updated by ONE thread only (the SLAM worker
//    thread for most probes).  snapshot() does a relaxed atomic read —
//    no locks, no contention.  If a probe IS accessed from multiple
//    threads (e.g. feed→output latency), use the atomic variant.
//
// ═══════════════════════════════════════════════════════════════════════════
//  Performance Targets
// ═══════════════════════════════════════════════════════════════════════════
//
//  ┌────────────────────────────────────────────────────────────────────────┐
//  │ Metric                   │  Drone (Jetson Orin)  │  Ground (x86 i7)  │
//  ├──────────────────────────┼───────────────────────┼────────────────────┤
//  │ IMU propagation          │  < 5 µs / sample      │  < 2 µs / sample  │
//  │ Scan deskew (10K pts)    │  < 2 ms               │  < 1 ms           │
//  │ ESIKF update (10K pts)   │  < 15 ms              │  < 8 ms           │
//  │ ikd-Tree insert          │  < 5 ms               │  < 3 ms           │
//  │ ikd-Tree search          │  < 8 ms               │  < 4 ms           │
//  │ End-to-end (feed→output) │  < 30 ms              │  < 15 ms          │
//  │ Total per-scan budget    │  < 80 ms (12.5 Hz)    │  < 40 ms (25 Hz)  │
//  │ Memory (steady state)    │  < 200 MB             │  < 400 MB         │
//  │ CPU load (single core)   │  < 80%                │  < 50%            │
//  │ Drift @ 1 km             │  < 1.0% (10 m)        │  < 0.5% (5 m)    │
//  │ Drift @ 1 km (with vis.) │  < 0.3% (3 m)         │  < 0.2% (2 m)    │
//  └──────────────────────────┴───────────────────────┴────────────────────┘
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#ifdef _MSC_VER
#include <intrin.h>
#endif

namespace thunderbird::odom {

// ═════════════════════════════════════════════════════════════════════════════
//  Compile-time gate
// ═════════════════════════════════════════════════════════════════════════════

#ifndef THUNDERBIRD_SLAM_PROFILING
#define THUNDERBIRD_SLAM_PROFILING 0
#endif

// ═════════════════════════════════════════════════════════════════════════════
//  Constants
// ═════════════════════════════════════════════════════════════════════════════

/// Maximum number of registered probes.
static constexpr size_t kMaxProbes = 32;

/// Number of histogram buckets (log₂-spaced, from 1 µs to ~1 s).
/// Bucket i covers [2^(i-1) µs, 2^i µs).  Bucket 0 = [0, 1) µs.
static constexpr size_t kHistogramBuckets = 21;

/// Cache line size for padding (x86 and ARMv8).
static constexpr size_t kCacheLineBytes = 64;

/// Maximum probe name length (including NUL terminator).
static constexpr size_t kMaxProbeName = 48;

// ═════════════════════════════════════════════════════════════════════════════
//  High-resolution clock
// ═════════════════════════════════════════════════════════════════════════════

/// Nanosecond timestamp from the cheapest available clock.
/// On x86: steady_clock (which g++ maps to rdtsc via clock_gettime).
/// On ARM: cntvct_el0 via steady_clock.
/// Cost: 15–25 ns per call on modern hardware.
inline int64_t profiler_now_ns() noexcept {
    return std::chrono::steady_clock::now().time_since_epoch().count();
}

// ═════════════════════════════════════════════════════════════════════════════
//  ProbeAccumulator — per-probe statistics (single writer, multiple readers)
// ═════════════════════════════════════════════════════════════════════════════

struct alignas(kCacheLineBytes) ProbeAccumulator {
    // ── Identity ────────────────────────────────────────────────────────
    char                              name[kMaxProbeName]{};
    std::atomic<bool>                 enabled{true};

    // ── Counters (written by one thread only) ───────────────────────────
    uint64_t                          count{0};
    int64_t                           sum_ns{0};
    int64_t                           min_ns{INT64_MAX};
    int64_t                           max_ns{0};

    // ── Histogram: bucket[i] counts samples in [2^(i-1), 2^i) µs ──────
    std::array<uint64_t, kHistogramBuckets> histogram{};

    // ── Atomic variant for multi-threaded probes ────────────────────────
    /// These mirror count/sum for probes that are fed from multiple threads.
    /// The non-atomic fields above are used when only one thread writes.
    std::atomic<uint64_t>            atomic_count{0};
    std::atomic<int64_t>             atomic_sum_ns{0};

    // ── Methods ─────────────────────────────────────────────────────────

    /// Record a single measurement (single-writer fast path).
    void record(int64_t elapsed_ns) noexcept {
        ++count;
        sum_ns += elapsed_ns;
        if (elapsed_ns < min_ns) min_ns = elapsed_ns;
        if (elapsed_ns > max_ns) max_ns = elapsed_ns;

        // Histogram bucket = floor(log2(elapsed_µs)) + 1, clamped.
        const int64_t us = elapsed_ns / 1000;
        if (us <= 0) {
            ++histogram[0];
        } else {
#ifdef _MSC_VER
            unsigned long idx;
            _BitScanReverse64(&idx, static_cast<unsigned __int64>(us));
            const int bucket = static_cast<int>(idx) + 1;
#else
            // __builtin_clzll(0) is UB — guarded above.
            const int bucket = 63 - __builtin_clzll(static_cast<uint64_t>(us)) + 1;
#endif
            ++histogram[static_cast<size_t>(
                bucket < static_cast<int>(kHistogramBuckets)
                    ? bucket : static_cast<int>(kHistogramBuckets) - 1)];
        }
    }

    /// Record a single measurement (multi-writer atomic path).
    void record_atomic(int64_t elapsed_ns) noexcept {
        atomic_count.fetch_add(1, std::memory_order_relaxed);
        atomic_sum_ns.fetch_add(elapsed_ns, std::memory_order_relaxed);
    }

    /// Reset all counters.
    void reset() noexcept {
        count  = 0;
        sum_ns = 0;
        min_ns = INT64_MAX;
        max_ns = 0;
        histogram.fill(0);
        atomic_count.store(0, std::memory_order_relaxed);
        atomic_sum_ns.store(0, std::memory_order_relaxed);
    }

    /// Mean latency in nanoseconds.
    [[nodiscard]] double mean_ns() const noexcept {
        return count > 0 ? static_cast<double>(sum_ns) / static_cast<double>(count)
                         : 0.0;
    }

    /// Mean latency in microseconds.
    [[nodiscard]] double mean_us() const noexcept { return mean_ns() / 1000.0; }

    /// Mean latency in milliseconds.
    [[nodiscard]] double mean_ms() const noexcept { return mean_ns() / 1.0e6; }

    /// Approximate p50 / p95 / p99 from histogram (µs).
    [[nodiscard]] double percentile_us(double pct) const noexcept {
        if (count == 0) return 0.0;
        const uint64_t target = static_cast<uint64_t>(
            static_cast<double>(count) * pct / 100.0);
        uint64_t cumulative = 0;
        for (size_t i = 0; i < kHistogramBuckets; ++i) {
            cumulative += histogram[i];
            if (cumulative >= target) {
                // Bucket i lower-bound in µs
                return (i == 0) ? 0.5 : static_cast<double>(uint64_t{1} << (i - 1));
            }
        }
        return static_cast<double>(uint64_t{1} << (kHistogramBuckets - 2));
    }
};

// ═════════════════════════════════════════════════════════════════════════════
//  ProbeSnapshot — immutable copy of one probe's state
// ═════════════════════════════════════════════════════════════════════════════

struct ProbeSnapshot {
    std::string name;
    uint64_t    count{0};
    double      mean_us{0};
    double      min_us{0};
    double      max_us{0};
    double      p50_us{0};
    double      p95_us{0};
    double      p99_us{0};
    double      total_ms{0};
    std::array<uint64_t, kHistogramBuckets> histogram{};
};

// ═════════════════════════════════════════════════════════════════════════════
//  MemoryMetrics — RSS / VmSize / allocator stats
// ═════════════════════════════════════════════════════════════════════════════

struct MemoryMetrics {
    size_t rss_bytes{0};               ///< resident set size
    size_t vm_bytes{0};                ///< virtual memory size
    size_t map_points{0};              ///< ikd-Tree point count
    size_t map_bytes_estimate{0};      ///< estimated map memory
    size_t ring_buffer_bytes{0};       ///< total ring buffer allocation
};

// ═════════════════════════════════════════════════════════════════════════════
//  CpuMetrics — per-thread and process CPU usage
// ═════════════════════════════════════════════════════════════════════════════

struct CpuMetrics {
    double process_cpu_pct{0};          ///< total process CPU%
    double worker_thread_cpu_pct{0};    ///< SLAM worker thread CPU%
    double ipc_thread_cpu_pct{0};       ///< IPC publisher thread CPU%
    double wall_time_s{0};              ///< elapsed wall time since start
    double user_time_s{0};              ///< user-mode CPU time
    double system_time_s{0};            ///< kernel-mode CPU time
};

// ═════════════════════════════════════════════════════════════════════════════
//  DriftMetrics — odometry drift measurement
// ═════════════════════════════════════════════════════════════════════════════

struct DriftMetrics {
    double distance_traveled_m{0};      ///< cumulative path length
    double absolute_position_error_m{0};///< ATE (if ground truth available)
    double relative_drift_pct{0};       ///< (ATE / distance) × 100
    double max_position_error_m{0};     ///< worst-case position error
    double mean_rotation_error_deg{0};  ///< mean rotation error
    size_t num_ground_truth_points{0};  ///< GT alignment count
};

// ═════════════════════════════════════════════════════════════════════════════
//  ProfileSnapshot — complete profiling report at a point in time
// ═════════════════════════════════════════════════════════════════════════════

struct ProfileSnapshot {
    int64_t                    timestamp_ns{0};
    std::vector<ProbeSnapshot> probes;
    MemoryMetrics              memory;
    CpuMetrics                 cpu;
    DriftMetrics               drift;
    double                     uptime_s{0};
};

// ═════════════════════════════════════════════════════════════════════════════
//  Ground truth pose for drift evaluation
// ═════════════════════════════════════════════════════════════════════════════

struct GroundTruthPose {
    int64_t              timestamp_ns{0};
    std::array<double,3> position{};             ///< metres, world frame
    std::array<double,4> quaternion{1,0,0,0};    ///< [w,x,y,z] body→world
};

// ═════════════════════════════════════════════════════════════════════════════
//  SlamProfiler — central profiling registry
// ═════════════════════════════════════════════════════════════════════════════
//
// Usage:
//
//   // At startup:
//   auto& prof = SlamProfiler::instance();
//   prof.reset();
//
//   // In instrumented code:
//   SLAM_PROFILE_SCOPE("esikf_update");
//   ... expensive work ...
//   // dtor records elapsed time
//
//   // Or manual start/stop:
//   auto t0 = profiler_now_ns();
//   ... work ...
//   prof.record("esikf_update", profiler_now_ns() - t0);
//
//   // Periodic reporting (e.g. every 10 s from health thread):
//   auto snap = prof.snapshot();
//   std::string json = prof.report_json(snap);
//
// ─────────────────────────────────────────────────────────────────────────────

class SlamProfiler {
public:
    /// Singleton access (lazy-init, thread-safe in C++11+).
    static SlamProfiler& instance() noexcept {
        static SlamProfiler s_instance;
        return s_instance;
    }

    // ── Probe registration ──────────────────────────────────────────────

    /// Get or create a probe by name.  Returns a stable pointer valid
    /// for the process lifetime.  Thread-safe (uses atomic probe_count_).
    ///
    /// @param name  Probe name (max kMaxProbeName-1 chars, truncated).
    /// @return Pointer to the probe accumulator, or nullptr if full.
    ProbeAccumulator* probe(std::string_view name) noexcept {
        // Linear scan — fine for ≤32 probes, called rarely.
        const size_t n = probe_count_.load(std::memory_order_acquire);
        for (size_t i = 0; i < n; ++i) {
            if (std::string_view(probes_[i].name) == name) {
                return &probes_[i];
            }
        }
        // Register new probe (CAS loop for thread safety).
        size_t slot = n;
        while (!probe_count_.compare_exchange_weak(
                   slot, slot + 1,
                   std::memory_order_acq_rel,
                   std::memory_order_acquire)) {
            // Re-check if someone else registered our name.
            for (size_t i = 0; i < slot; ++i) {
                if (std::string_view(probes_[i].name) == name)
                    return &probes_[i];
            }
            if (slot >= kMaxProbes) return nullptr;
        }
        if (slot >= kMaxProbes) return nullptr;

        auto& p = probes_[slot];
        const size_t len = name.size() < kMaxProbeName - 1
                             ? name.size() : kMaxProbeName - 1;
        std::memcpy(p.name, name.data(), len);
        p.name[len] = '\0';
        p.reset();
        return &p;
    }

    // ── Direct record (no scoped timer) ─────────────────────────────────

    /// Record a measurement by probe name.
    void record(std::string_view name, int64_t elapsed_ns) noexcept {
        if (auto* p = probe(name); p && p->enabled.load(std::memory_order_relaxed))
            p->record(elapsed_ns);
    }

    /// Record a measurement atomically (multi-threaded probes).
    void record_atomic(std::string_view name, int64_t elapsed_ns) noexcept {
        if (auto* p = probe(name); p && p->enabled.load(std::memory_order_relaxed))
            p->record_atomic(elapsed_ns);
    }

    // ── Enable / disable individual probes ──────────────────────────────

    void enable(std::string_view name) noexcept {
        if (auto* p = probe(name)) p->enabled.store(true, std::memory_order_relaxed);
    }

    void disable(std::string_view name) noexcept {
        if (auto* p = probe(name)) p->enabled.store(false, std::memory_order_relaxed);
    }

    void enable_all() noexcept {
        const size_t n = probe_count_.load(std::memory_order_acquire);
        for (size_t i = 0; i < n; ++i)
            probes_[i].enabled.store(true, std::memory_order_relaxed);
    }

    void disable_all() noexcept {
        const size_t n = probe_count_.load(std::memory_order_acquire);
        for (size_t i = 0; i < n; ++i)
            probes_[i].enabled.store(false, std::memory_order_relaxed);
    }

    // ── Snapshot ────────────────────────────────────────────────────────

    /// Capture a consistent snapshot of all probes + system metrics.
    /// This is a read-only operation — safe to call from any thread.
    [[nodiscard]] ProfileSnapshot snapshot() const noexcept;

    // ── System metrics (populated by snapshot()) ────────────────────────

    /// Manually set memory metrics (call from health thread).
    void update_memory(const MemoryMetrics& m) noexcept { memory_ = m; }

    /// Manually set CPU metrics.
    void update_cpu(const CpuMetrics& c) noexcept { cpu_ = c; }

    /// Manually set drift metrics.
    void update_drift(const DriftMetrics& d) noexcept { drift_ = d; }

    // ── Drift evaluation ────────────────────────────────────────────────

    /// Feed a ground-truth pose for drift computation.
    /// Call from the benchmark harness, not the SLAM engine.
    void feed_ground_truth(const GroundTruthPose& gt) noexcept;

    /// Feed an estimated pose (from SLAM output) matched to ground truth.
    void feed_estimated_pose(int64_t timestamp_ns,
                             const std::array<double,3>& position,
                             const std::array<double,4>& quaternion) noexcept;

    /// Compute drift metrics from accumulated GT/estimated pairs.
    /// Result is stored internally and included in next snapshot().
    void compute_drift() noexcept;

    // ── Report generation ───────────────────────────────────────────────

    /// Generate a JSON report string from a snapshot.
    [[nodiscard]] static std::string report_json(const ProfileSnapshot& snap);

    /// Generate a CSV report (one row per probe).
    [[nodiscard]] static std::string report_csv(const ProfileSnapshot& snap);

    /// Generate a human-readable text report.
    [[nodiscard]] static std::string report_text(const ProfileSnapshot& snap);

    // ── Reset ───────────────────────────────────────────────────────────

    /// Reset all probe data (does not unregister probes).
    void reset() noexcept {
        const size_t n = probe_count_.load(std::memory_order_acquire);
        for (size_t i = 0; i < n; ++i)
            probes_[i].reset();
        memory_ = {};
        cpu_    = {};
        drift_  = {};
        gt_poses_.clear();
        est_poses_.clear();
        start_time_ns_ = profiler_now_ns();
    }

    // ── Accessors ───────────────────────────────────────────────────────

    [[nodiscard]] size_t probe_count() const noexcept {
        return probe_count_.load(std::memory_order_acquire);
    }

    [[nodiscard]] const ProbeAccumulator& probe_at(size_t i) const noexcept {
        return probes_[i];
    }

private:
    SlamProfiler() noexcept : start_time_ns_(profiler_now_ns()) {}
    ~SlamProfiler() = default;
    SlamProfiler(const SlamProfiler&) = delete;
    SlamProfiler& operator=(const SlamProfiler&) = delete;

    // ── Storage ─────────────────────────────────────────────────────────

    std::array<ProbeAccumulator, kMaxProbes> probes_{};
    std::atomic<size_t>                      probe_count_{0};

    MemoryMetrics memory_{};
    CpuMetrics    cpu_{};
    DriftMetrics  drift_{};

    int64_t       start_time_ns_{0};

    // ── Ground truth / estimated trajectory buffers ─────────────────────
    struct TimestampedPosition {
        int64_t              timestamp_ns{0};
        std::array<double,3> position{};
        std::array<double,4> quaternion{1,0,0,0};
    };

    std::vector<TimestampedPosition> gt_poses_;
    std::vector<TimestampedPosition> est_poses_;
};

// ═════════════════════════════════════════════════════════════════════════════
//  ScopedTimer — RAII timing guard
// ═════════════════════════════════════════════════════════════════════════════

class ScopedTimer {
public:
    explicit ScopedTimer(ProbeAccumulator* probe) noexcept
        : probe_(probe), start_(profiler_now_ns()) {}

    ~ScopedTimer() noexcept {
        if (probe_ && probe_->enabled.load(std::memory_order_relaxed))
            probe_->record(profiler_now_ns() - start_);
    }

    /// Elapsed nanoseconds so far (for intermediate checks).
    [[nodiscard]] int64_t elapsed_ns() const noexcept {
        return profiler_now_ns() - start_;
    }

    // Non-copyable.
    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;

private:
    ProbeAccumulator* probe_;
    int64_t           start_;
};

/// Atomic variant for multi-threaded probes.
class ScopedTimerAtomic {
public:
    explicit ScopedTimerAtomic(ProbeAccumulator* probe) noexcept
        : probe_(probe), start_(profiler_now_ns()) {}

    ~ScopedTimerAtomic() noexcept {
        if (probe_ && probe_->enabled.load(std::memory_order_relaxed))
            probe_->record_atomic(profiler_now_ns() - start_);
    }

    ScopedTimerAtomic(const ScopedTimerAtomic&) = delete;
    ScopedTimerAtomic& operator=(const ScopedTimerAtomic&) = delete;

private:
    ProbeAccumulator* probe_;
    int64_t           start_;
};

// ═════════════════════════════════════════════════════════════════════════════
//  Macros — zero-overhead when THUNDERBIRD_SLAM_PROFILING == 0
// ═════════════════════════════════════════════════════════════════════════════

/// Concatenation helpers for unique variable names.
#define SLAM_PROF_CONCAT_(a, b) a##b
#define SLAM_PROF_CONCAT(a, b) SLAM_PROF_CONCAT_(a, b)

#if THUNDERBIRD_SLAM_PROFILING

/// Scope-based timing.  Place at the top of a function or block.
/// Example:
///   void esikf_update() {
///       SLAM_PROFILE_SCOPE("esikf_update");
///       // ... work measured here ...
///   }
#define SLAM_PROFILE_SCOPE(name)                                              \
    static auto* SLAM_PROF_CONCAT(_slam_probe_, __LINE__) =                   \
        ::thunderbird::odom::SlamProfiler::instance().probe(name);            \
    ::thunderbird::odom::ScopedTimer SLAM_PROF_CONCAT(_slam_timer_, __LINE__)(\
        SLAM_PROF_CONCAT(_slam_probe_, __LINE__))

/// Multi-threaded scope timer (uses atomic accumulation).
#define SLAM_PROFILE_SCOPE_MT(name)                                           \
    static auto* SLAM_PROF_CONCAT(_slam_probe_mt_, __LINE__) =                \
        ::thunderbird::odom::SlamProfiler::instance().probe(name);            \
    ::thunderbird::odom::ScopedTimerAtomic                                    \
        SLAM_PROF_CONCAT(_slam_timer_mt_, __LINE__)(                          \
            SLAM_PROF_CONCAT(_slam_probe_mt_, __LINE__))

/// Manual start: returns a nanosecond timestamp.
#define SLAM_PROFILE_START() ::thunderbird::odom::profiler_now_ns()

/// Manual stop: records elapsed time to a named probe.
#define SLAM_PROFILE_STOP(name, start_ns)                                     \
    ::thunderbird::odom::SlamProfiler::instance().record(                     \
        (name), ::thunderbird::odom::profiler_now_ns() - (start_ns))

/// Record a pre-computed duration.
#define SLAM_PROFILE_RECORD(name, elapsed_ns)                                 \
    ::thunderbird::odom::SlamProfiler::instance().record((name), (elapsed_ns))

#else // THUNDERBIRD_SLAM_PROFILING == 0

#define SLAM_PROFILE_SCOPE(name)               ((void)0)
#define SLAM_PROFILE_SCOPE_MT(name)            ((void)0)
#define SLAM_PROFILE_START()                   int64_t{0}
#define SLAM_PROFILE_STOP(name, start_ns)      ((void)0)
#define SLAM_PROFILE_RECORD(name, elapsed_ns)  ((void)0)

#endif // THUNDERBIRD_SLAM_PROFILING

// ═════════════════════════════════════════════════════════════════════════════
//  Benchmark Configuration
// ═════════════════════════════════════════════════════════════════════════════
//
// Configuration for the benchmark harness (test_slam_benchmark).
// ─────────────────────────────────────────────────────────────────────────────

/// Parameters for a single benchmark scenario.
struct BenchmarkScenario {
    std::string name;                       ///< e.g. "urban_drone_500m"
    std::string dataset_path;               ///< path to recorded .tbrec file
    std::string ground_truth_path;          ///< path to GT trajectory (TUM format)

    // ── Sensor configuration ────────────────────────────────────────────
    double imu_rate_hz{400.0};
    double lidar_rate_hz{10.0};
    size_t points_per_scan{10000};

    // ── Duration / distance ─────────────────────────────────────────────
    double expected_distance_m{0};          ///< expected total path length
    double duration_s{0};                   ///< dataset duration (0 = full)

    // ── Pass/fail thresholds ────────────────────────────────────────────
    double max_end_to_end_ms{30.0};
    double max_imu_propagation_us{5.0};
    double max_esikf_update_ms{15.0};
    double max_memory_mb{200.0};
    double max_cpu_pct{80.0};
    double max_drift_pct{1.0};
};

/// Collection of scenarios for a full benchmark run.
struct BenchmarkPlan {
    std::string                     name;       ///< e.g. "v0.2.0 regression"
    std::vector<BenchmarkScenario>  scenarios;
};

// ═════════════════════════════════════════════════════════════════════════════
//  Dataset Test Plan — predefined scenarios
// ═════════════════════════════════════════════════════════════════════════════
//
// The following factory functions return BenchmarkPlan structs for standard
// validation scenarios.  Actual dataset files are expected under:
//   ${THUNDERBIRD_DATA_DIR}/benchmark/
//
// ─────────────────────────────────────────────────────────────────────────────

/// Standard drone benchmark plan.
///
///  Scenario         Distance   Env        IMU     LiDAR    Points   Drift
///  ─────────────────────────────────────────────────────────────────────────
///  hover_static     0 m        indoor     400 Hz  10 Hz    10K      N/A
///  figure_eight     100 m      outdoor    400 Hz  10 Hz    15K      < 0.5%
///  urban_fast       500 m      urban      400 Hz  10 Hz    20K      < 1.0%
///  long_range       1000 m     mixed      400 Hz  10 Hz    20K      < 1.0%
///  aggressive_yaw   200 m      outdoor    1000 Hz 10 Hz    10K      < 1.5%
///  degenerate_hall  300 m      corridor   400 Hz  10 Hz    5K       < 2.0%
///
BenchmarkPlan benchmark_plan_drone();

/// Standard ground vehicle benchmark plan.
///
///  Scenario         Distance   Env        IMU     LiDAR    Points   Drift
///  ─────────────────────────────────────────────────────────────────────────
///  parking_lot      200 m      outdoor    200 Hz  10 Hz    30K      < 0.3%
///  campus_loop      1000 m     mixed      200 Hz  10 Hz    30K      < 0.5%
///  highway          2000 m     highway    200 Hz  10 Hz    40K      < 0.5%
///  urban_dense      1000 m     urban      200 Hz  10 Hz    50K      < 0.5%
///  forest_trail     500 m      forest     200 Hz  10 Hz    20K      < 0.8%
///  garage_multi     300 m      indoor     200 Hz  10 Hz    25K      < 0.5%
///
BenchmarkPlan benchmark_plan_ground_vehicle();

/// Stress test plan (performance limits).
///
///  Scenario            Points/scan   Map size    Duration
///  ────────────────────────────────────────────────────────
///  max_throughput       100K          500K        60 s
///  memory_pressure      50K           1M          300 s
///  sustained_1hr        20K           500K        3600 s
///
BenchmarkPlan benchmark_plan_stress();

} // namespace thunderbird::odom
