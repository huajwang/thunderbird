// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Evaluation Framework: PerfTimer non-inline impls
// ─────────────────────────────────────────────────────────────────────────────

#include "eval/perf_timer.h"

#include <chrono>
#include <cstdio>
#include <iostream>

namespace eval {

// ─────────────────────────────────────────────────────────────────────────────
//  ReplayRateLogger
// ─────────────────────────────────────────────────────────────────────────────

bool ReplayRateLogger::tick(size_t frame_index, size_t total_frames_expected) {
    frames_since_log_++;
    total_frames_++;

    const auto now = PerfTimer::now();
    const double dt = std::chrono::duration<double>(now - last_log_).count();

    if (dt < interval_s_) return false;

    const double fps = static_cast<double>(frames_since_log_) / dt;
    std::fprintf(stderr, "[replay] frame %zu | %.1f fps",
                 frame_index, fps);
    if (total_frames_expected > 0) {
        const double pct = 100.0 * static_cast<double>(frame_index)
                                 / static_cast<double>(total_frames_expected);
        std::fprintf(stderr, " | %.0f%%", pct);
    }
    std::fprintf(stderr, "\n");

    last_log_ = now;
    frames_since_log_ = 0;
    return true;
}

void ReplayRateLogger::finish() {
    const double fps = overall_fps();
    std::fprintf(stderr, "[replay] done: %zu frames in %.1f s (%.1f fps)\n",
                 total_frames_,
                 std::chrono::duration<double>(PerfTimer::now() - start_).count(),
                 fps);
}

double ReplayRateLogger::overall_fps() const {
    const double dt = std::chrono::duration<double>(PerfTimer::now() - start_).count();
    return dt > 0.0 ? static_cast<double>(total_frames_) / dt : 0.0;
}

} // namespace eval
