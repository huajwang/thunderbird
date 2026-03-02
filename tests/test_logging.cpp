// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Logging system unit tests
// ─────────────────────────────────────────────────────────────────────────────
//
// Tests cover:
//   1. init() idempotence (second call is a no-op)
//   2. is_initialized() reflects state
//   3. Per-module level setting
//   4. Global level setting
//   5. get() returns non-null for all modules after init
//   6. get_by_index() bounds checking
//   7. Macro compilation (TB_LOG_INFO, TB_LOG_WARN_ONCE, TB_LOG_DEBUG_EVERY_N)
//   8. shutdown() nulls loggers and get() returns nullptr
//
// ─────────────────────────────────────────────────────────────────────────────

#include "thunderbird/logging.h"
#include "thunderbird/log_macros.h"

#include <cstdio>
#include <cstdlib>

namespace tbl = thunderbird::logging;

#define VERIFY(expr) \
    do { if (!(expr)) { std::fprintf(stderr, "FAIL: %s @ %s:%d\n", #expr, __FILE__, __LINE__); std::abort(); } } while (0)

// ─── Tests ──────────────────────────────────────────────────────────────────

static void test_init_idempotence() {
    std::printf("  init idempotence ... ");
    tbl::LoggingConfig cfg;
    cfg.console_enabled = false;   // suppress test output
    cfg.file_enabled    = false;
    tbl::init(cfg);

    VERIFY(tbl::is_initialized());

    // Second call should be a no-op (no crash, no duplicate loggers).
    tbl::init(cfg);

    VERIFY(tbl::is_initialized());
    std::printf("OK\n");
}

static void test_get_returns_non_null() {
    std::printf("  get() non-null ... ");
    for (std::size_t i = 0; i < tbl::kModuleCount; ++i) {
        auto* logger = tbl::get(static_cast<tbl::Module>(i));
        VERIFY(logger != nullptr);
    }
    std::printf("OK\n");
}

static void test_get_by_index_bounds() {
    std::printf("  get_by_index() bounds ... ");
    VERIFY(tbl::get_by_index(0) != nullptr);
    VERIFY(tbl::get_by_index(static_cast<uint8_t>(tbl::kModuleCount)) == nullptr);
    VERIFY(tbl::get_by_index(255) == nullptr);
    std::printf("OK\n");
}

static void test_per_module_level() {
    std::printf("  per-module level ... ");
    tbl::set_module_level(tbl::Module::Transport, spdlog::level::err);
    auto* logger = tbl::get(tbl::Module::Transport);
    VERIFY(logger->level() == spdlog::level::err);

    // Reset
    tbl::set_module_level(tbl::Module::Transport, spdlog::level::info);
    VERIFY(logger->level() == spdlog::level::info);
    std::printf("OK\n");
}

static void test_global_level() {
    std::printf("  global level ... ");
    tbl::set_level(spdlog::level::warn);
    for (std::size_t i = 0; i < tbl::kModuleCount; ++i) {
        auto* logger = tbl::get(static_cast<tbl::Module>(i));
        VERIFY(logger->level() == spdlog::level::warn);
    }
    // Reset
    tbl::set_level(spdlog::level::info);
    std::printf("OK\n");
}

static void test_macros_compile_and_run() {
    std::printf("  macros compile & run ... ");

    // These just need to not crash — output goes to /dev/null since
    // console is disabled in our test config.
    TB_LOG_TRACE(tbl::Module::Core,       "trace {}", 1);
    TB_LOG_DEBUG(tbl::Module::Core,       "debug {}", 2);
    TB_LOG_INFO(tbl::Module::Core,        "info {}",  3);
    TB_LOG_WARN(tbl::Module::Transport,   "warn {}",  4);
    TB_LOG_ERROR(tbl::Module::Parser,     "error {}", 5);
    TB_LOG_CRITICAL(tbl::Module::Slam,    "crit {}",  6);

    TB_LOG_WARN_ONCE(tbl::Module::Core,   "once {}",  7);
    TB_LOG_WARN_ONCE(tbl::Module::Core,   "once {}",  8);  // should not fire

    TB_LOG_DEBUG_EVERY_N(tbl::Module::Core, 10, "every-10 {}", 9);

    std::printf("OK\n");
}

static void test_flush_no_crash() {
    std::printf("  flush() ... ");
    tbl::flush();
    std::printf("OK\n");
}

static void test_shutdown() {
    std::printf("  shutdown() ... ");
    // After shutdown, get() must return nullptr for every module.
    tbl::shutdown();
    for (std::size_t i = 0; i < tbl::kModuleCount; ++i) {
        VERIFY(tbl::get(static_cast<tbl::Module>(i)) == nullptr);
    }
    // Macros must not crash even after shutdown (they null-check internally).
    TB_LOG_INFO(tbl::Module::Core, "after shutdown {}", 99);
    std::printf("OK\n");
}

// ─────────────────────────────────────────────────────────────────────────────

int main() {
    std::printf("test_logging\n");

    test_init_idempotence();
    test_get_returns_non_null();
    test_get_by_index_bounds();
    test_per_module_level();
    test_global_level();
    test_macros_compile_and_run();
    test_flush_no_crash();
    test_shutdown();  // Must be last — destroys loggers

    std::printf("All logging tests passed.\n");
    return 0;
}
