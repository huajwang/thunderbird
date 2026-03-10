// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird Firmware — Platform clock abstraction
// ─────────────────────────────────────────────────────────────────────────────
//
// Provides a monotonic nanosecond clock used for packet timestamps.
// On real hardware, replace the implementation with a high-resolution
// timer peripheral (e.g. ARM DWT cycle counter or dedicated PTP clock).
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/// Return current monotonic time in nanoseconds.
/// On stub/POSIX builds this uses clock_gettime(CLOCK_MONOTONIC).
uint64_t fw_clock_now_ns(void);

#ifdef __cplusplus
}
#endif
