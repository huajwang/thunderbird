// ─────────────────────────────────────────────────────────────────────────────
// Test — RuntimeProfiler & ProfileSnapshot
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/odom/runtime_profiler.h"
#include "thunderbird/odom/slam_types.h"

#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

using namespace thunderbird::odom;

// ── Helpers ─────────────────────────────────────────────────────────────────

/// Spin-wait for approximately `us` microseconds (avoids sleep granularity).
static void burn_us(double us) {
    const auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::duration<double, std::micro>(
               std::chrono::steady_clock::now() - t0).count() < us) {
        // busy wait
    }
}

// ── Tests ───────────────────────────────────────────────────────────────────

/// Suppress GCC -Wunused-but-set-variable for assert()-only usage.
#define USE(x) ((void)(x))

/// Fresh profiler produces a zero-state snapshot.
static void test_empty_snapshot() {
    RuntimeProfiler prof;
    ProfileSnapshot snap = prof.snapshot();

    for (size_t i = 0; i < kProfileModuleCount; ++i) {
        assert(snap.modules[i].invocations == 0);
        assert(snap.modules[i].avg_us == 0.0);
        assert(snap.modules[i].total_ms == 0.0);
    }

    assert(snap.total_scans == 0);
    assert(snap.total_imu_samples == 0);
    assert(snap.worker_utilization_pct == 0.0);
    assert(snap.wall_time_s >= 0.0);
    USE(snap);

    std::puts("  empty_snapshot:           PASS");
}

/// RAII Scope increments invocation count and records non-zero timing.
static void test_scope_records() {
    RuntimeProfiler prof;

    // Record 3 scopes for EsikfUpdate.
    for (int i = 0; i < 3; ++i) {
        RuntimeProfiler::Scope s(prof, ProfileModule::EsikfUpdate);
        burn_us(50);
    }

    ProfileSnapshot snap = prof.snapshot();
    const auto& m = snap.modules[static_cast<size_t>(ProfileModule::EsikfUpdate)];

    assert(m.invocations == 3);
    assert(m.avg_us > 0.0);
    assert(m.min_us > 0.0);
    assert(m.max_us >= m.min_us);
    assert(m.total_ms > 0.0);
    USE(m);

    // Other modules should be untouched.
    assert(snap.modules[static_cast<size_t>(ProfileModule::Deskew)].invocations == 0);
    USE(snap);

    std::puts("  scope_records:           PASS");
}

/// record() directly (without Scope) also works.
static void test_manual_record() {
    RuntimeProfiler prof;

    auto t0 = RuntimeProfiler::Clock::now();
    burn_us(100);
    prof.record(ProfileModule::ImuPropagate, t0);

    t0 = RuntimeProfiler::Clock::now();
    burn_us(100);
    prof.record(ProfileModule::ImuPropagate, t0);

    ProfileSnapshot snap = prof.snapshot();
    assert(snap.modules[static_cast<size_t>(ProfileModule::ImuPropagate)].invocations == 2);
    assert(snap.total_imu_samples == 2);
    USE(snap);

    std::puts("  manual_record:           PASS");
}

/// reset() clears all accumulated statistics.
static void test_reset_clears() {
    RuntimeProfiler prof;

    // Accumulate some data.
    {
        RuntimeProfiler::Scope s(prof, ProfileModule::EsikfUpdate);
        burn_us(50);
    }
    {
        RuntimeProfiler::Scope s(prof, ProfileModule::ScanTotal);
        burn_us(50);
    }
    auto t0 = RuntimeProfiler::Clock::now();
    burn_us(50);
    prof.record(ProfileModule::ImuPropagate, t0);

    ProfileSnapshot before = prof.snapshot();
    assert(before.modules[static_cast<size_t>(ProfileModule::EsikfUpdate)].invocations == 1);
    assert(before.total_scans == 1);
    assert(before.total_imu_samples == 1);
    USE(before);

    prof.reset();

    ProfileSnapshot after = prof.snapshot();
    for (size_t i = 0; i < kProfileModuleCount; ++i) {
        assert(after.modules[i].invocations == 0);
    }
    assert(after.total_scans == 0);
    assert(after.total_imu_samples == 0);
    assert(after.worker_utilization_pct == 0.0);
    USE(after);

    std::puts("  reset_clears:            PASS");
}

/// Multiple modules can be recorded independently.
static void test_multiple_modules() {
    RuntimeProfiler prof;

    {
        RuntimeProfiler::Scope s(prof, ProfileModule::Deskew);
        burn_us(30);
    }
    {
        RuntimeProfiler::Scope s(prof, ProfileModule::IkdInsert);
        burn_us(30);
    }
    {
        RuntimeProfiler::Scope s(prof, ProfileModule::IkdInsert);
        burn_us(30);
    }

    ProfileSnapshot snap = prof.snapshot();
    assert(snap.modules[static_cast<size_t>(ProfileModule::Deskew)].invocations == 1);
    assert(snap.modules[static_cast<size_t>(ProfileModule::IkdInsert)].invocations == 2);
    USE(snap);

    std::puts("  multiple_modules:        PASS");
}

/// ScanTotal drives total_scans and scan latency summary fields.
static void test_scan_total_latency() {
    RuntimeProfiler prof;

    for (int i = 0; i < 5; ++i) {
        RuntimeProfiler::Scope s(prof, ProfileModule::ScanTotal);
        burn_us(200);
    }

    ProfileSnapshot snap = prof.snapshot();
    assert(snap.total_scans == 5);
    assert(snap.avg_scan_latency_ms > 0.0);
    assert(snap.max_scan_latency_ms >= snap.avg_scan_latency_ms);
    // Percentiles should be populated.
    assert(snap.p50_scan_latency_ms > 0.0);
    assert(snap.p95_scan_latency_ms >= snap.p50_scan_latency_ms);
    assert(snap.p99_scan_latency_ms >= snap.p95_scan_latency_ms);
    USE(snap);

    std::puts("  scan_total_latency:      PASS");
}

/// Worker utilisation is derived from wall_time − idle_time.
static void test_worker_utilisation() {
    RuntimeProfiler prof;

    // Simulate busy work.
    {
        RuntimeProfiler::Scope s(prof, ProfileModule::EsikfUpdate);
        burn_us(500);
    }

    // Simulate idle period.
    prof.markIdleStart();
    burn_us(500);
    prof.markIdleEnd();

    ProfileSnapshot snap = prof.snapshot();
    // Utilisation should be roughly 50% (busy ~500 µs, idle ~500 µs).
    assert(snap.worker_utilization_pct > 20.0);
    assert(snap.worker_utilization_pct < 80.0);
    USE(snap);

    std::puts("  worker_utilisation:      PASS");
}

/// sampleMemory() populates RSS fields.
static void test_sample_memory() {
    RuntimeProfiler prof;

    prof.sampleMemory();

    ProfileSnapshot snap = prof.snapshot();
    // On supported platforms, RSS should be non-zero.
    // On unsupported platforms, query_rss_bytes() returns 0 — that's fine.
#if defined(__linux__) || defined(__CYGWIN__) || defined(_WIN32) || defined(__APPLE__)
    assert(snap.current_rss_bytes > 0);
    assert(snap.peak_rss_bytes > 0);
    assert(snap.peak_rss_bytes >= snap.current_rss_bytes);
#endif
    USE(snap);

    std::puts("  sample_memory:           PASS");
}

/// wall_time_s advances over time.
static void test_wall_time() {
    RuntimeProfiler prof;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ProfileSnapshot snap = prof.snapshot();
    assert(snap.wall_time_s >= 0.005);  // at least 5 ms
    USE(snap);

    std::puts("  wall_time:               PASS");
}

/// cpu_pct is computed relative to ScanTotal for sub-modules.
static void test_cpu_pct() {
    RuntimeProfiler prof;

    // Record ScanTotal encompassing sub-modules.
    auto scan_start = RuntimeProfiler::Clock::now();
    {
        RuntimeProfiler::Scope s(prof, ProfileModule::EsikfUpdate);
        burn_us(300);
    }
    {
        RuntimeProfiler::Scope s(prof, ProfileModule::IkdInsert);
        burn_us(100);
    }
    prof.record(ProfileModule::ScanTotal, scan_start);

    ProfileSnapshot snap = prof.snapshot();

    // EsikfUpdate should be ~75%, IkdInsert ~25% of ScanTotal.
    const auto& esikf = snap.modules[static_cast<size_t>(ProfileModule::EsikfUpdate)];
    const auto& ikd   = snap.modules[static_cast<size_t>(ProfileModule::IkdInsert)];

    assert(esikf.cpu_pct > 0.0);
    assert(ikd.cpu_pct > 0.0);
    assert(esikf.cpu_pct > ikd.cpu_pct);
    USE(esikf); USE(ikd);

    std::puts("  cpu_pct:                 PASS");
}

/// Snapshot is safely callable from a different thread while recording.
static void test_concurrent_snapshot() {
    RuntimeProfiler prof;

    // Writer thread: record scopes in a loop.
    std::atomic<bool> stop{false};
    std::thread writer([&] {
        while (!stop.load(std::memory_order_relaxed)) {
            RuntimeProfiler::Scope s(prof, ProfileModule::Deskew);
            burn_us(10);
        }
    });

    // Reader thread (this one): take snapshots concurrently.
    for (int i = 0; i < 50; ++i) {
        ProfileSnapshot snap = prof.snapshot();
        // Must not crash; invocations should be non-negative.
        (void)snap;
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }

    stop.store(true, std::memory_order_relaxed);
    writer.join();

    // Final snapshot should have accumulated some invocations.
    ProfileSnapshot final_snap = prof.snapshot();
    assert(final_snap.modules[static_cast<size_t>(ProfileModule::Deskew)].invocations > 0);
    USE(final_snap);

    std::puts("  concurrent_snapshot:     PASS");
}

/// RollingStats percentile produces sensible values on sorted input.
static void test_percentile_values() {
    RuntimeProfiler prof;

    // Record 100 samples with known ascending values (1..100 µs).
    for (int i = 1; i <= 100; ++i) {
        // Use record() with a crafted time span.  We fake it by recording
        // ImuPropagate durations directly.
        auto t0 = RuntimeProfiler::Clock::now();
        burn_us(static_cast<double>(i));
        prof.record(ProfileModule::ImuPropagate, t0);
    }

    ProfileSnapshot snap = prof.snapshot();
    const auto& m = snap.modules[static_cast<size_t>(ProfileModule::ImuPropagate)];

    assert(m.invocations == 100);
    // p50 should be roughly 50 µs, p95 ~95 µs, p99 ~99 µs.
    // Allow wide tolerance due to burn_us imprecision.
    assert(m.p50_us > 10.0);   // should be around 50
    assert(m.p95_us > m.p50_us);
    assert(m.p99_us >= m.p95_us);
    assert(m.max_us >= m.p99_us);
    USE(m);

    std::puts("  percentile_values:       PASS");
}

/// Module names are non-null and correctly indexed.
static void test_module_names() {
    RuntimeProfiler prof;
    ProfileSnapshot snap = prof.snapshot();

    for (size_t i = 0; i < kProfileModuleCount; ++i) {
        assert(snap.modules[i].name != nullptr);
        assert(snap.modules[i].name[0] != '\0');
    }

    // Spot-check specific names.
    assert(std::string(snap.modules[static_cast<size_t>(
        ProfileModule::EsikfUpdate)].name) == "esikf_update");
    assert(std::string(snap.modules[static_cast<size_t>(
        ProfileModule::WorkerIdle)].name) == "worker_idle");
    USE(snap);

    std::puts("  module_names:            PASS");
}

// ── Main ────────────────────────────────────────────────────────────────────

int main() {
    std::puts("RuntimeProfiler tests:");

    test_empty_snapshot();
    test_scope_records();
    test_manual_record();
    test_reset_clears();
    test_multiple_modules();
    test_scan_total_latency();
    test_worker_utilisation();
    test_sample_memory();
    test_wall_time();
    test_cpu_pct();
    test_concurrent_snapshot();
    test_percentile_values();
    test_module_names();

    std::puts("\nRuntimeProfiler: ALL TESTS PASSED");
    return 0;
}
