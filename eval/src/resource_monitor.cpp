// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Evaluation Framework: Resource Monitor
// ─────────────────────────────────────────────────────────────────────────────

#include "eval/resource_monitor.h"
#include "eval/perf_timer.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <numeric>
#include <string>
#include <thread>

namespace eval {

// ─────────────────────────────────────────────────────────────────────────────
//  Linux /proc helpers
// ─────────────────────────────────────────────────────────────────────────────

#ifdef __linux__

#include <unistd.h>

static size_t read_rss_bytes() {
    std::ifstream f("/proc/self/statm");
    if (!f.is_open()) return 0;
    size_t vm_pages = 0, rss_pages = 0;
    f >> vm_pages >> rss_pages;
    return rss_pages * 4096;  // page size assumption (safe on most Linux)
}

struct ProcTimes {
    double user_s{0};
    double system_s{0};
};

static ProcTimes read_proc_times() {
    ProcTimes pt;
    std::ifstream f("/proc/self/stat");
    if (!f.is_open()) return pt;

    std::string tok;
    // Fields: pid comm state ppid ... (14th=utime, 15th=stime in clock ticks)
    for (int i = 0; i < 13; ++i) f >> tok;
    long utime = 0, stime = 0;
    f >> utime >> stime;

    const double ticks_per_sec = static_cast<double>(sysconf(_SC_CLK_TCK));
    pt.user_s   = static_cast<double>(utime) / ticks_per_sec;
    pt.system_s = static_cast<double>(stime) / ticks_per_sec;
    return pt;
}

#else  // Non-Linux stub

static size_t read_rss_bytes() { return 0; }

struct ProcTimes { double user_s{0}; double system_s{0}; };
static ProcTimes read_proc_times() { return {}; }

#endif

// ─────────────────────────────────────────────────────────────────────────────
//  ResourceMonitor implementation
// ─────────────────────────────────────────────────────────────────────────────

void ResourceMonitor::start() {
    if (running_.exchange(true)) return;  // already running
    samples_.clear();
    thread_ = std::thread(&ResourceMonitor::run, this);
}

void ResourceMonitor::stop() {
    running_.store(false);
    if (thread_.joinable()) thread_.join();
}

void ResourceMonitor::run() {
    const auto t0 = PerfTimer::now();
    auto prev_times = read_proc_times();
    auto prev_wall  = t0;

    while (running_.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        const auto now        = PerfTimer::now();
        const auto wall_delta = std::chrono::duration<double>(now - prev_wall).count();
        const auto cur_times  = read_proc_times();

        ResourceSample s;
        s.wall_time_s = std::chrono::duration<double>(now - t0).count();
        s.rss_bytes   = read_rss_bytes();

        if (wall_delta > 0.0) {
            const double cpu_delta = (cur_times.user_s - prev_times.user_s) +
                                     (cur_times.system_s - prev_times.system_s);
            s.cpu_pct = (cpu_delta / wall_delta) * 100.0;
        }

        samples_.push_back(s);
        prev_times = cur_times;
        prev_wall  = now;
    }
}

size_t ResourceMonitor::peakRss() const {
    size_t peak = 0;
    for (const auto& s : samples_) {
        if (s.rss_bytes > peak) peak = s.rss_bytes;
    }
    return peak;
}

double ResourceMonitor::avgCpu() const {
    if (samples_.empty()) return 0.0;
    double sum = 0.0;
    for (const auto& s : samples_) sum += s.cpu_pct;
    return sum / static_cast<double>(samples_.size());
}

} // namespace eval
