// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Zero-cost logging macros
// ─────────────────────────────────────────────────────────────────────────────
//
// Two-stage gate ensures zero overhead when logging is disabled:
//
//   1. Compile-time gate  — `if constexpr` on THUNDERBIRD_LOG_LEVEL_COMPILE.
//      When a level is below the compile threshold the ENTIRE macro body
//      (including argument evaluation) is elided by the compiler.  Ship
//      release builds with -DTHUNDERBIRD_LOG_LEVEL_COMPILE=2 (info) to
//      strip all TRACE and DEBUG sites at zero binary cost.
//
//   2. Runtime gate       — spdlog::logger::should_log() is a single relaxed
//      atomic load (~1 ns).  The [[unlikely]] hint tells the CPU branch
//      predictor that the slow formatting path is rare.
//
// Usage:
//   TB_LOG_INFO(Module::Transport, "connected to {}", uri);
//   TB_LOG_ERROR(Module::Device, "stall detected: {} ms", stall_ms);
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/logging.h"

#include <mutex>
#include <spdlog/spdlog.h>

// ─── Compile-time minimum log level ─────────────────────────────────────────
// Override at build time:  cmake -DTHUNDERBIRD_LOG_LEVEL_COMPILE=2 ..
//
// Values:  0 = trace (all enabled, default)
//          1 = debug
//          2 = info
//          3 = warn
//          4 = error
//          5 = critical
//          6 = off   (all logging stripped)

#ifndef THUNDERBIRD_LOG_LEVEL_COMPILE
#define THUNDERBIRD_LOG_LEVEL_COMPILE 0
#endif

// ─── Internal macro ─────────────────────────────────────────────────────────

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define TB_LOG_IMPL_(level_val, level_enum, mod, ...)                          \
    do {                                                                        \
        if constexpr ((level_val) >= THUNDERBIRD_LOG_LEVEL_COMPILE) {           \
            auto* _tb_lg_ = ::thunderbird::logging::get(mod);                   \
            if (_tb_lg_ && _tb_lg_->should_log(level_enum)) [[unlikely]] {      \
                _tb_lg_->log(                                                   \
                    ::spdlog::source_loc{__FILE__, __LINE__, __func__},          \
                    level_enum, __VA_ARGS__);                                    \
            }                                                                   \
        }                                                                       \
    } while (0)

// ─── Public macros ──────────────────────────────────────────────────────────

// NOLINTBEGIN(cppcoreguidelines-macro-usage)

#define TB_LOG_TRACE(mod, ...) \
    TB_LOG_IMPL_(0, ::spdlog::level::trace,    mod, __VA_ARGS__)

#define TB_LOG_DEBUG(mod, ...) \
    TB_LOG_IMPL_(1, ::spdlog::level::debug,    mod, __VA_ARGS__)

#define TB_LOG_INFO(mod, ...) \
    TB_LOG_IMPL_(2, ::spdlog::level::info,     mod, __VA_ARGS__)

#define TB_LOG_WARN(mod, ...) \
    TB_LOG_IMPL_(3, ::spdlog::level::warn,     mod, __VA_ARGS__)

#define TB_LOG_ERROR(mod, ...) \
    TB_LOG_IMPL_(4, ::spdlog::level::err,      mod, __VA_ARGS__)

#define TB_LOG_CRITICAL(mod, ...) \
    TB_LOG_IMPL_(5, ::spdlog::level::critical, mod, __VA_ARGS__)

// NOLINTEND(cppcoreguidelines-macro-usage)

// ─── Conditional logging (rate-limited / once) ──────────────────────────────

/// Log at most once (e.g. stub warnings).
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define TB_LOG_ONCE(level_val, level_enum, mod, ...)                           \
    do {                                                                        \
        static std::once_flag _tb_once_flag_;                                   \
        std::call_once(_tb_once_flag_, [&]() {                                  \
            TB_LOG_IMPL_(level_val, level_enum, mod, __VA_ARGS__);              \
        });                                                                     \
    } while (0)

#define TB_LOG_WARN_ONCE(mod, ...)  TB_LOG_ONCE(3, ::spdlog::level::warn, mod, __VA_ARGS__)
#define TB_LOG_ERROR_ONCE(mod, ...) TB_LOG_ONCE(4, ::spdlog::level::err,  mod, __VA_ARGS__)

/// Log at most every N calls (e.g. hot-path diagnostics at reduced rate).
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define TB_LOG_EVERY_N(level_val, level_enum, mod, n, ...)                     \
    do {                                                                        \
        static uint64_t _tb_log_ctr_ = 0;                                      \
        if ((++_tb_log_ctr_) % (n) == 0) {                                     \
            TB_LOG_IMPL_(level_val, level_enum, mod, __VA_ARGS__);              \
        }                                                                       \
    } while (0)

#define TB_LOG_DEBUG_EVERY_N(mod, n, ...) \
    TB_LOG_EVERY_N(1, ::spdlog::level::debug, mod, n, __VA_ARGS__)

#define TB_LOG_TRACE_EVERY_N(mod, n, ...) \
    TB_LOG_EVERY_N(0, ::spdlog::level::trace, mod, n, __VA_ARGS__)
