// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Structured logging system (spdlog)
// ─────────────────────────────────────────────────────────────────────────────
//
// Provides per-module loggers with:
//   • Compile-time log stripping  (THUNDERBIRD_LOG_LEVEL_COMPILE)
//   • Runtime-configurable levels (per-module or global)
//   • Async logging option        (bounded MPSC queue, drop on overflow)
//   • File rotation               (size-based, configurable count)
//   • JSON structured output      (for fleet telemetry ingestion)
//   • Zero allocation in hot path when logging is disabled
//
// Quick start:
//   thunderbird::logging::init();                       // defaults
//   TB_LOG_INFO(Module::Transport, "connected to {}", uri);
//
// Production build (strip trace + debug at compile time):
//   cmake -DTHUNDERBIRD_LOG_LEVEL_COMPILE=2 ..
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <spdlog/spdlog.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "thunderbird/export.h"
#include "thunderbird/abi.h"

namespace thunderbird::logging {
THUNDERBIRD_ABI_NAMESPACE_BEGIN

// ─── Module identifiers ─────────────────────────────────────────────────────
// Each module gets its own spdlog::logger so levels can be tuned independently.

enum class Module : uint8_t {
    Core       = 0,   // "tb.core"       — DeviceManager, general SDK
    Transport  = 1,   // "tb.transport"  — Ethernet, USB, Simulated
    Parser     = 2,   // "tb.parser"     — PacketParser, PacketDecoder, VLP-16
    TimeSync   = 3,   // "tb.timesync"   — ClockService, TimeSync, SyncEngine
    Slam       = 4,   // "tb.slam"       — SlamEngine, SlamHealth, SlamProfiler
    Perception = 5,   // "tb.perception" — PerceptionEngine, detectors
    Device     = 6,   // "tb.device"     — DeviceHealthMonitor
    Recorder   = 7,   // "tb.recorder"   — Recorder, Player
    Assembler  = 8,   // "tb.assembler"  — LidarFrameAssembler
    Count_
};

/// Number of module loggers.
inline constexpr std::size_t kModuleCount =
    static_cast<std::size_t>(Module::Count_);

/// Human-readable name for each module (used as spdlog logger name).
inline constexpr const char* kModuleNames[kModuleCount] = {
    "tb.core",
    "tb.transport",
    "tb.parser",
    "tb.timesync",
    "tb.slam",
    "tb.perception",
    "tb.device",
    "tb.recorder",
    "tb.assembler",
};

// ─── Configuration ──────────────────────────────────────────────────────────

struct LoggingConfig {
    /// Default runtime log level applied to all modules.
    spdlog::level::level_enum level = spdlog::level::info;

    // ── Async ───────────────────────────────────────────────────────────
    /// Enable asynchronous logging (background thread for I/O).
    bool   async            = false;
    /// Queue depth for async mode (must be power of 2).
    size_t async_queue_size = 8192;
    /// Number of background worker threads (typically 1).
    size_t async_threads    = 1;

    // ── Console ─────────────────────────────────────────────────────────
    bool console_enabled = true;
    bool console_color   = true;

    // ── Rotating file ───────────────────────────────────────────────────
    bool        file_enabled  = false;
    std::string file_path     = "thunderbird.log";
    size_t      max_file_size = 50 * 1024 * 1024;   // 50 MB
    size_t      max_files     = 5;

    // ── JSON structured log (fleet telemetry) ───────────────────────────
    bool        json_enabled  = false;
    std::string json_path     = "thunderbird.json.log";
    size_t      json_max_size = 100 * 1024 * 1024;   // 100 MB
    size_t      json_max_files = 10;

    // ── Pattern overrides (empty = use defaults) ────────────────────────
    /// Console / file pattern  (default: "[%Y-%m-%d %H:%M:%S.%e] [%l] [%n] %v")
    std::string pattern;
    /// JSON pattern (default: R"({"ts":"%Y-%m-%dT%H:%M:%S.%eZ","level":"%l","module":"%n","msg":"%v"})")
    std::string json_pattern;
};

// ─── Public API ─────────────────────────────────────────────────────────────

/// Initialize all module loggers with the given configuration.
/// Safe to call multiple times — subsequent calls are no-ops after the first.
/// NOT thread-safe with respect to itself; call once from main().
void init(const LoggingConfig& cfg = {});

/// Returns true if init() has been called.
[[nodiscard]] bool is_initialized() noexcept;

/// Change the runtime log level of ALL module loggers.
void set_level(spdlog::level::level_enum level);

/// Change the runtime log level of a single module logger.
void set_module_level(Module mod, spdlog::level::level_enum level);

/// Flush all sinks synchronously.  Call before shutdown or crash handler.
void flush();

/// Drop all loggers and shut down async thread pool (if any).
/// After this call, get() returns nullptr for all modules and
/// logging macros become no-ops.
void shutdown();

/// Retrieve the spdlog logger for a module.
/// Returns nullptr if logging has not been initialized via init()
/// or after shutdown() has been called.
[[nodiscard]] spdlog::logger* get(Module mod) noexcept;

/// Convenience: retrieve logger by index.
[[nodiscard]] inline spdlog::logger* get_by_index(uint8_t idx) noexcept {
    if (idx >= kModuleCount) return nullptr;
    return get(static_cast<Module>(idx));
}

THUNDERBIRD_ABI_NAMESPACE_END
}  // namespace thunderbird::logging
