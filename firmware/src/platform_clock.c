// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird Firmware — Platform clock (POSIX stub)
// ─────────────────────────────────────────────────────────────────────────────
//
// On real hardware, replace with a high-resolution timer read (e.g. ARM
// DWT cycle counter scaled to nanoseconds, or a PTP hardware clock).
// ─────────────────────────────────────────────────────────────────────────────
#include "fw/platform_clock.h"

#ifdef _WIN32
#include <windows.h>

static uint64_t s_freq = 0;

uint64_t fw_clock_now_ns(void) {
    if (s_freq == 0) {
        LARGE_INTEGER f;
        QueryPerformanceFrequency(&f);
        s_freq = (uint64_t)f.QuadPart;
    }
    LARGE_INTEGER cnt;
    QueryPerformanceCounter(&cnt);
    // Convert to nanoseconds: cnt * 1e9 / freq
    return (uint64_t)((double)cnt.QuadPart * 1e9 / (double)s_freq);
}

#else
#include <time.h>

uint64_t fw_clock_now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

#endif
