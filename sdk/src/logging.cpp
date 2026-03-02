// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Logging system implementation
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/logging.h"

#include <spdlog/spdlog.h>
#include <spdlog/async.h>
#include <spdlog/pattern_formatter.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>

#include <atomic>
#include <mutex>
#include <vector>

namespace thunderbird::logging {
THUNDERBIRD_ABI_NAMESPACE_BEGIN

// ─── Static state ───────────────────────────────────────────────────────────

static std::atomic<bool>   g_initialized{false};
static std::once_flag      g_init_flag;

// Raw pointer array for zero-indirection access in the hot path.
// Lifetime managed by the shared_ptrs in g_loggers_owned below.
static spdlog::logger*     g_loggers[kModuleCount] = {};

// Shared ownership kept alive until shutdown().
static std::shared_ptr<spdlog::logger> g_loggers_owned[kModuleCount];

// ─── Default patterns ───────────────────────────────────────────────────────

static constexpr const char* kDefaultPattern =
    "[%Y-%m-%d %H:%M:%S.%e] [%l] [%n] %v";

static constexpr const char* kDefaultJsonPattern =
    R"({"ts":"%Y-%m-%dT%H:%M:%S.%e","level":"%l","module":"%n"})";

// ─── Helpers ────────────────────────────────────────────────────────────────

/// Build the shared sink vector from config.  Non-JSON sinks get the
/// console/file pattern applied; JSON sinks get their own structured pattern.
static std::vector<spdlog::sink_ptr>
build_sinks(const LoggingConfig& cfg,
            const std::string& pattern,
            const std::string& json_pattern) {
    std::vector<spdlog::sink_ptr> sinks;
    sinks.reserve(3);

    // Console sink
    if (cfg.console_enabled) {
        spdlog::sink_ptr console;
        if (cfg.console_color) {
            console = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        } else {
            console = std::make_shared<spdlog::sinks::stdout_sink_mt>();
        }
        console->set_pattern(pattern);
        sinks.push_back(std::move(console));
    }

    // Rotating file sink
    if (cfg.file_enabled && !cfg.file_path.empty()) {
        auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            cfg.file_path,
            cfg.max_file_size,
            cfg.max_files);
        file_sink->set_pattern(pattern);
        sinks.push_back(std::move(file_sink));
    }

    // JSON rotating file sink (separate pattern)
    if (cfg.json_enabled && !cfg.json_path.empty()) {
        auto json_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            cfg.json_path,
            cfg.json_max_size,
            cfg.json_max_files);
        // Use UTC time for JSON logs so timestamps are unambiguous.
        json_sink->set_formatter(
            std::make_unique<spdlog::pattern_formatter>(
                json_pattern, spdlog::pattern_time_type::utc));
        sinks.push_back(std::move(json_sink));
    }

    return sinks;
}

/// Create a single module logger (sync or async).
/// Patterns are already set per-sink in build_sinks().
static std::shared_ptr<spdlog::logger>
make_logger(const char* name,
            const std::vector<spdlog::sink_ptr>& sinks,
            spdlog::level::level_enum level,
            bool async_mode) {

    std::shared_ptr<spdlog::logger> logger;

    if (async_mode) {
        logger = std::make_shared<spdlog::async_logger>(
            name,
            sinks.begin(), sinks.end(),
            spdlog::thread_pool(),
            spdlog::async_overflow_policy::overrun_oldest);
    } else {
        logger = std::make_shared<spdlog::logger>(
            name, sinks.begin(), sinks.end());
    }

    logger->set_level(level);

    // Flush on warnings and above so that crash dumps contain recent output.
    logger->flush_on(spdlog::level::warn);

    // Register with spdlog's global registry for external access.
    spdlog::register_logger(logger);

    return logger;
}

// ─── Public API implementation ──────────────────────────────────────────────

void init(const LoggingConfig& cfg) {
    std::call_once(g_init_flag, [&cfg]() {
        const std::string pattern =
            cfg.pattern.empty() ? kDefaultPattern : cfg.pattern;
        const std::string json_pattern =
            cfg.json_pattern.empty() ? kDefaultJsonPattern : cfg.json_pattern;

        // Initialize async thread pool if requested.
        if (cfg.async) {
            spdlog::init_thread_pool(cfg.async_queue_size, cfg.async_threads);
        }

        // Build shared sinks (patterns applied per-sink).
        auto sinks = build_sinks(cfg, pattern, json_pattern);

        // Create per-module loggers.
        for (std::size_t i = 0; i < kModuleCount; ++i) {
            g_loggers_owned[i] = make_logger(
                kModuleNames[i], sinks, cfg.level, cfg.async);
            g_loggers[i] = g_loggers_owned[i].get();
        }

        g_initialized.store(true, std::memory_order_release);
    });
}

bool is_initialized() noexcept {
    return g_initialized.load(std::memory_order_acquire);
}

void set_level(spdlog::level::level_enum level) {
    for (std::size_t i = 0; i < kModuleCount; ++i) {
        if (g_loggers[i]) {
            g_loggers[i]->set_level(level);
        }
    }
}

void set_module_level(Module mod, spdlog::level::level_enum level) {
    const auto idx = static_cast<std::size_t>(mod);
    if (idx < kModuleCount && g_loggers[idx]) {
        g_loggers[idx]->set_level(level);
    }
}

void flush() {
    for (std::size_t i = 0; i < kModuleCount; ++i) {
        if (g_loggers[i]) {
            g_loggers[i]->flush();
        }
    }
}

void shutdown() {
    for (std::size_t i = 0; i < kModuleCount; ++i) {
        g_loggers[i] = nullptr;
        g_loggers_owned[i].reset();
    }
    spdlog::shutdown();
    // Note: g_initialized stays true; re-init is not supported.
}

spdlog::logger* get(Module mod) noexcept {
    return g_loggers[static_cast<std::size_t>(mod)];
}

THUNDERBIRD_ABI_NAMESPACE_END
}  // namespace thunderbird::logging
