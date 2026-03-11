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
    // Convert to nanoseconds using integer arithmetic to avoid precision loss:
    // ns = (ticks / freq) * 1e9 + (ticks % freq) * 1e9 / freq
    const uint64_t ticks   = (uint64_t)cnt.QuadPart;
    const uint64_t seconds = ticks / s_freq;
    const uint64_t rem     = ticks % s_freq;
    return seconds * 1000000000ULL + (rem * 1000000000ULL) / s_freq;
}

#elif defined(FW_BARE_METAL)

// Bare-metal stub — returns 0 until replaced with a HW timer read
static volatile uint64_t s_tick_ns = 0;

uint64_t fw_clock_now_ns(void) {
    return s_tick_ns;
}

void fw_clock_set_ns(uint64_t ns) {
    s_tick_ns = ns;
}

#else
#include <time.h>

uint64_t fw_clock_now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

#endif
