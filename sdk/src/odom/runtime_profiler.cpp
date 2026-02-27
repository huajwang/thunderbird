// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — RuntimeProfiler implementation
// ─────────────────────────────────────────────────────────────────────────────

#include "thunderbird/odom/runtime_profiler.h"

#include <algorithm>
#include <chrono>
#include <cstdio>

// ─────────────────────────────────────────────────────────────────────────────
//  Platform-specific RSS query
// ─────────────────────────────────────────────────────────────────────────────

#if defined(__linux__) || defined(__CYGWIN__)

#include <fstream>
#include <string>
#include <unistd.h>

static size_t query_rss_bytes() {
    std::ifstream f("/proc/self/statm");
    if (!f.is_open()) return 0;
    size_t vm_pages = 0, rss_pages = 0;
    f >> vm_pages >> rss_pages;
    return rss_pages * static_cast<size_t>(sysconf(_SC_PAGESIZE));
}

#elif defined(_WIN32)

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <psapi.h>

static size_t query_rss_bytes() {
    PROCESS_MEMORY_COUNTERS pmc;
    if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc))) {
        return static_cast<size_t>(pmc.WorkingSetSize);
    }
    return 0;
}

#elif defined(__APPLE__)

#include <mach/mach.h>

static size_t query_rss_bytes() {
    mach_task_basic_info_data_t info;
    mach_msg_type_number_t count = MACH_TASK_BASIC_INFO_COUNT;
    if (task_info(mach_task_self(), MACH_TASK_BASIC_INFO,
                  reinterpret_cast<task_info_t>(&info), &count) == KERN_SUCCESS) {
        return static_cast<size_t>(info.resident_size);
    }
    return 0;
}

#else

static size_t query_rss_bytes() { return 0; }

#endif

// ─────────────────────────────────────────────────────────────────────────────

namespace thunderbird::odom {

void RuntimeProfiler::sampleMemory() noexcept {
    const size_t rss = query_rss_bytes();
    std::lock_guard<std::mutex> lk(mu_);
    current_rss_ = rss;
    if (current_rss_ > peak_rss_) {
        peak_rss_ = current_rss_;
    }
}

ProfileSnapshot RuntimeProfiler::snapshot() const {
    ProfileSnapshot snap{};

    const auto now = Clock::now();

    // ── Copy raw data under the lock ────────────────────────────────────
    // We memcpy the module accumulators and scalar state, then release the
    // mutex *before* computing percentiles (which allocate + sort).  This
    // keeps lock hold time proportional to a flat copy (~460 KB memcpy)
    // rather than O(N log N) sorting per module.
    // start_time_ is also read here because reset() writes it under mu_.
    std::array<RollingStats, kProfileModuleCount> mod_copy;
    int64_t idle_ns_copy;
    size_t  peak_rss_copy;
    size_t  curr_rss_copy;
    TimePoint start_copy;

    {
        std::lock_guard<std::mutex> lk(mu_);
        mod_copy      = modules_;
        idle_ns_copy  = total_idle_ns_;
        peak_rss_copy = peak_rss_;
        curr_rss_copy = current_rss_;
        start_copy    = start_time_;
    }

    snap.wall_time_s = std::chrono::duration<double>(
        now - start_copy).count();

    // ── Per-module statistics (lock released) ───────────────────────────
    const double scan_total_us =
        mod_copy[static_cast<size_t>(ProfileModule::ScanTotal)].sum();

    for (size_t i = 0; i < kProfileModuleCount; ++i) {
        const auto& rs = mod_copy[i];
        auto& mp       = snap.modules[i];

        mp.name        = kModuleNames[i];
        mp.invocations = rs.count();
        mp.avg_us      = rs.mean();
        mp.min_us      = rs.min_val();
        mp.max_us      = rs.max_val();
        mp.total_ms    = rs.sum() / 1000.0;

        if (rs.count() > 0) {
            rs.percentiles(mp.p50_us, mp.p95_us, mp.p99_us);
        }

        // CPU% relative to ScanTotal (only for sub-modules).
        if (scan_total_us > 0.0 &&
            static_cast<ProfileModule>(i) != ProfileModule::ScanTotal &&
            static_cast<ProfileModule>(i) != ProfileModule::WorkerIdle &&
            static_cast<ProfileModule>(i) != ProfileModule::ImuPropagate) {
            mp.cpu_pct = (rs.sum() / scan_total_us) * 100.0;
        }
    }

    // ── Worker utilisation ──────────────────────────────────────────────
    // Derive busy time as wall_time − idle_time.  This avoids double-
    // counting from nested scopes (e.g. ScanTotal wrapping EsikfUpdate).
    // Only meaningful once at least one idle period has been observed.
    const int64_t wall_ns = std::chrono::duration_cast<
        std::chrono::nanoseconds>(now - start_copy).count();
    if (wall_ns > 0 && idle_ns_copy > 0) {
        const int64_t busy_ns = wall_ns - idle_ns_copy;
        snap.worker_utilization_pct =
            std::max(0.0, static_cast<double>(busy_ns) /
                          static_cast<double>(wall_ns) * 100.0);
    }

    // ── Memory ──────────────────────────────────────────────────────────
    snap.peak_rss_bytes    = peak_rss_copy;
    snap.current_rss_bytes = curr_rss_copy;

    // ── Scan latency summary (from ScanTotal module) ────────────────────
    const auto& st = mod_copy[static_cast<size_t>(ProfileModule::ScanTotal)];
    snap.total_scans = st.count();
    if (st.count() > 0) {
        snap.avg_scan_latency_ms = st.mean() / 1000.0;
        double p50_us, p95_us, p99_us;
        st.percentiles(p50_us, p95_us, p99_us);
        snap.p50_scan_latency_ms = p50_us / 1000.0;
        snap.p95_scan_latency_ms = p95_us / 1000.0;
        snap.p99_scan_latency_ms = p99_us / 1000.0;
        snap.max_scan_latency_ms = st.max_val() / 1000.0;
    }

    // ── IMU sample count ────────────────────────────────────────────────
    snap.total_imu_samples =
        mod_copy[static_cast<size_t>(ProfileModule::ImuPropagate)].count();

    return snap;
}

} // namespace thunderbird::odom
