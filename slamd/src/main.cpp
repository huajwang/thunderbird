// ─────────────────────────────────────────────────────────────────────────────
// acme_slamd — SLAM daemon entry point
// ─────────────────────────────────────────────────────────────────────────────
//
// Usage:
//   acme_slamd --config /etc/acme_slamd/config.yaml
//   acme_slamd --help
//   acme_slamd --version
//
// Signal handling:
//   SIGTERM / SIGINT  → graceful shutdown
//   SIGHUP            → reload config (log level, health thresholds)
//
// ─────────────────────────────────────────────────────────────────────────────
#include "acme_slamd.h"

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

// ─── Version info (from CMake) ──────────────────────────────────────────────
#ifndef SLAMD_VERSION
#define SLAMD_VERSION "0.1.4-dev"
#endif

// ─── Global daemon pointer for signal handlers ─────────────────────────────
static std::atomic<thunderbird::slamd::SlamDaemon*> g_daemon{nullptr};
static std::atomic<bool> g_reload_requested{false};
static std::string g_config_path;

// ─── Signal handlers ────────────────────────────────────────────────────────

extern "C" void handle_shutdown(int /*sig*/) {
    auto* d = g_daemon.load(std::memory_order_acquire);
    if (d) d->requestShutdown();
}

extern "C" void handle_reload(int /*sig*/) {
    g_reload_requested.store(true, std::memory_order_release);
}

// ─── CLI usage ──────────────────────────────────────────────────────────────

static void printUsage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " [OPTIONS]\n"
        << "\n"
        << "Options:\n"
        << "  -c, --config <path>   Path to config.yaml\n"
        << "                        Default: /etc/acme_slamd/config.yaml\n"
        << "  -v, --version         Print version and exit\n"
        << "  -h, --help            Print this help and exit\n"
        << "\n"
        << "Signals:\n"
        << "  SIGTERM, SIGINT       Graceful shutdown\n"
        << "  SIGHUP                Reload configuration\n"
        << "\n"
        << "Systemd:\n"
        << "  Designed as a Type=notify service.  Emits READY=1,\n"
        << "  WATCHDOG=1, STATUS=... and STOPPING=1 notifications.\n"
        << std::endl;
}

// ─── Main ───────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    // ── Parse CLI args ──────────────────────────────────────────────────
    g_config_path = "/etc/acme_slamd/config.yaml";

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];

        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            return 0;
        }
        if (arg == "-v" || arg == "--version") {
            std::cout << "acme_slamd " << SLAMD_VERSION << "\n";
            return 0;
        }
        if ((arg == "-c" || arg == "--config") && i + 1 < argc) {
            g_config_path = argv[++i];
            continue;
        }

        std::cerr << "Unknown option: " << arg << "\n";
        printUsage(argv[0]);
        return 1;
    }

    // ── Load configuration ──────────────────────────────────────────────
    thunderbird::slamd::DaemonConfig config;
    std::string error;
    if (!thunderbird::slamd::loadConfig(g_config_path, config, error)) {
        std::cerr << "ERROR: " << error << "\n";
        return 1;
    }

    // ── Install signal handlers ─────────────────────────────────────────
    {
        struct sigaction sa{};
        sa.sa_handler = handle_shutdown;
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = SA_RESTART;
        sigaction(SIGTERM, &sa, nullptr);
        sigaction(SIGINT, &sa, nullptr);

        struct sigaction sa_hup{};
        sa_hup.sa_handler = handle_reload;
        sigemptyset(&sa_hup.sa_mask);
        sa_hup.sa_flags = SA_RESTART;
        sigaction(SIGHUP, &sa_hup, nullptr);
    }

    // ── Construct and start daemon ──────────────────────────────────────
    thunderbird::slamd::SlamDaemon daemon(std::move(config));
    g_daemon.store(&daemon, std::memory_order_release);

    if (!daemon.start()) {
        std::cerr << "ERROR: daemon.start() failed\n";
        return 1;
    }

    // ── Main loop (handles SIGHUP reload) ───────────────────────────────
    // The daemon's run() blocks in a condvar.  We poll for SIGHUP in a
    // separate pattern: run() will unblock on SIGTERM/SIGINT via
    // requestShutdown().  For SIGHUP, we use a polling thread that checks
    // the atomic flag.
    //
    // Simpler approach: just run and check reload after wakeup.
    // The run() call blocks until requestShutdown() is called.

    std::thread reload_watcher([&daemon]() {
        while (daemon.isRunning()) {
            if (g_reload_requested.exchange(false,
                    std::memory_order_acq_rel)) {
                daemon.reloadConfig(g_config_path);
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });

    daemon.run();  // blocks until SIGTERM/SIGINT

    // ── Shutdown ────────────────────────────────────────────────────────
    g_daemon.store(nullptr, std::memory_order_release);
    daemon.stop();

    if (reload_watcher.joinable()) reload_watcher.join();

    std::cerr << "acme_slamd exited cleanly\n";
    return 0;
}
