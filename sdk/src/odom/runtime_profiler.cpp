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

static size_t query_rss_bytes() {
    std::ifstream f("/proc/self/statm");
    if (!f.is_open()) return 0;
    size_t vm_pages = 0, rss_pages = 0;
    f >> vm_pages >> rss_pages;
    return rss_pages * 4096;  // standard page size
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
    current_rss_ = query_rss_bytes();
    if (current_rss_ > peak_rss_) {
        peak_rss_ = current_rss_;
    }
}

ProfileSnapshot RuntimeProfiler::snapshot() const {
    ProfileSnapshot snap{};

    const auto now = Clock::now();
    snap.wall_time_s = std::chrono::duration<double>(
        now - start_time_).count();

    // ── Per-module statistics ───────────────────────────────────────────
    const double scan_total_us =
        modules_[static_cast<size_t>(ProfileModule::ScanTotal)].sum();

    for (size_t i = 0; i < kProfileModuleCount; ++i) {
        const auto& rs = modules_[i];
        auto& mp       = snap.modules[i];

        mp.name        = kModuleNames[i];
        mp.invocations = rs.count();
        mp.avg_us      = rs.mean();
        mp.min_us      = rs.min_val();
        mp.max_us      = rs.max_val();
        mp.total_ms    = rs.sum() / 1000.0;

        if (rs.count() > 0) {
            mp.p50_us = rs.percentile(50.0);
            mp.p95_us = rs.percentile(95.0);
            mp.p99_us = rs.percentile(99.0);
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
    const int64_t total_ns = total_busy_ns_ + total_idle_ns_;
    if (total_ns > 0) {
        snap.worker_utilization_pct =
            static_cast<double>(total_busy_ns_) /
            static_cast<double>(total_ns) * 100.0;
    }

    // ── Memory ──────────────────────────────────────────────────────────
    snap.peak_rss_bytes    = peak_rss_;
    snap.current_rss_bytes = current_rss_;

    // ── Scan latency summary (from ScanTotal module) ────────────────────
    const auto& st = modules_[static_cast<size_t>(ProfileModule::ScanTotal)];
    snap.total_scans = st.count();
    if (st.count() > 0) {
        snap.avg_scan_latency_ms = st.mean() / 1000.0;
        snap.p50_scan_latency_ms = st.percentile(50.0) / 1000.0;
        snap.p95_scan_latency_ms = st.percentile(95.0) / 1000.0;
        snap.p99_scan_latency_ms = st.percentile(99.0) / 1000.0;
        snap.max_scan_latency_ms = st.max_val() / 1000.0;
    }

    // ── IMU sample count ────────────────────────────────────────────────
    snap.total_imu_samples =
        modules_[static_cast<size_t>(ProfileModule::ImuPropagate)].count();

    return snap;
}

} // namespace thunderbird::odom
