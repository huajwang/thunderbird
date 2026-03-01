// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — DiagnosticsManager implementation
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/diagnostics.h"

#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <iomanip>
#include <sstream>
#include <thread>
#include <vector>

namespace {

/// Escape a string for JSON output (handles quotes, backslashes, control chars).
void json_escape(std::ostream& os, const std::string& s) {
    for (char c : s) {
        switch (c) {
            case '"':  os << "\\\""; break;
            case '\\': os << "\\\\"; break;
            case '\b': os << "\\b"; break;
            case '\f': os << "\\f"; break;
            case '\n': os << "\\n"; break;
            case '\r': os << "\\r"; break;
            case '\t': os << "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned char>(c));
                    os << buf;
                } else {
                    os << c;
                }
        }
    }
}

} // anonymous namespace

namespace thunderbird {
THUNDERBIRD_ABI_NAMESPACE_BEGIN

// ═════════════════════════════════════════════════════════════════════════════
//  DiagnosticsSnapshot helpers
// ═════════════════════════════════════════════════════════════════════════════

DiagnosticsSnapshot DiagnosticsSnapshot::compute_rates(
        const DiagnosticsSnapshot& previous,
        const DiagnosticsSnapshot& current) {

    using namespace std::chrono;
    const double elapsed_s = duration<double>(
        current.timestamp - previous.timestamp).count();

    DiagnosticsSnapshot result;
    result.timestamp = current.timestamp;

    if (elapsed_s <= 0.0) return current;  // degenerate — just return raw

    for (const auto& [group, metrics] : current.groups) {
        auto& dst = result.groups[group];
        for (const auto& [name, mv] : metrics) {
            if (mv.type == MetricType::Counter) {
                // Compute per-second rate from delta.
                double prev_val = 0.0;
                auto pit = previous.groups.find(group);
                if (pit != previous.groups.end()) {
                    auto mit = pit->second.find(name);
                    if (mit != pit->second.end())
                        prev_val = mit->second.value;
                }
                dst[name] = MetricValue{
                    MetricType::Gauge,  // rate is a gauge
                    (mv.value - prev_val) / elapsed_s
                };
                // Also store the raw counter with ".total" suffix.
                dst[name + ".total"] = mv;
            } else {
                dst[name] = mv;
            }
        }
    }
    return result;
}

std::string DiagnosticsSnapshot::to_json() const {
    std::ostringstream os;
    os << '{';

    bool first_group = true;
    for (const auto& [group, metrics] : groups) {
        if (!first_group) os << ',';
        first_group = false;

        os << '"';
        json_escape(os, group);
        os << "\":{";
        bool first_metric = true;
        for (const auto& [name, mv] : metrics) {
            if (!first_metric) os << ',';
            first_metric = false;

            os << '"';
            json_escape(os, name);
            os << "\":";
            if (mv.type == MetricType::Flag) {
                os << (mv.value != 0.0 ? "true" : "false");
            } else {
                if (mv.type == MetricType::Counter) {
                    os << static_cast<uint64_t>(mv.value);
                } else {
                    // Fixed-point for gauges to avoid scientific notation.
                    os << std::fixed << std::setprecision(6) << mv.value;
                }
            }
        }
        os << '}';
    }

    os << '}';
    return os.str();
}

// ═════════════════════════════════════════════════════════════════════════════
//  PImpl
// ═════════════════════════════════════════════════════════════════════════════

struct DiagnosticsManager::Impl {
    DiagnosticsConfig config;

    // ── Collectors ──────────────────────────────────────────────────────
    struct NamedCollector {
        std::string name;
        CollectorFn fn;
    };
    std::mutex                    collectors_mu;
    std::vector<NamedCollector>   collectors;

    // ── Latest snapshot ─────────────────────────────────────────────────
    mutable std::mutex            snap_mu;
    DiagnosticsSnapshot           latest;
    DiagnosticsSnapshot           previous;  // for rate computation

    // ── Thread control ──────────────────────────────────────────────────
    std::atomic<bool>             running{false};
    std::thread                   thread;
    std::mutex                    cv_mu;
    std::condition_variable       cv;

    // ── Collection loop ─────────────────────────────────────────────────
    void run() {
        while (running.load(std::memory_order_relaxed)) {
            // Collect from all registered collectors.
            DiagnosticsSnapshot snap;
            snap.timestamp = std::chrono::steady_clock::now();

            // Copy the collector list under lock, then invoke outside
            // the lock so that slow collectors don't block register_collector().
            std::vector<NamedCollector> local_collectors;
            {
                std::lock_guard<std::mutex> lk(collectors_mu);
                local_collectors = collectors;
            }
            for (const auto& c : local_collectors) {
                try {
                    snap.groups[c.name] = c.fn();
                } catch (...) {
                    // Collector threw — skip this group this cycle.
                }
            }

            // Optionally compute rates.
            DiagnosticsSnapshot to_store;
            {
                std::lock_guard<std::mutex> lk(snap_mu);
                if (config.compute_rates && !previous.empty()) {
                    to_store = DiagnosticsSnapshot::compute_rates(previous, snap);
                } else {
                    to_store = snap;
                }
                previous = snap;               // raw values for next delta
                latest   = std::move(to_store);
            }

            // Wait for next interval or stop signal.
            {
                std::unique_lock<std::mutex> lk(cv_mu);
                cv.wait_for(lk, config.interval, [this] {
                    return !running.load(std::memory_order_relaxed);
                });
            }
        }
    }
};

// ═════════════════════════════════════════════════════════════════════════════
//  DiagnosticsManager public API
// ═════════════════════════════════════════════════════════════════════════════

DiagnosticsManager::DiagnosticsManager(DiagnosticsConfig config)
    : impl_(std::make_unique<Impl>())
{
    impl_->config = std::move(config);
}

DiagnosticsManager::~DiagnosticsManager() {
    stop();
}

void DiagnosticsManager::register_collector(
        const std::string& name, CollectorFn fn) {
    std::lock_guard<std::mutex> lk(impl_->collectors_mu);
    impl_->collectors.push_back({name, std::move(fn)});
}

void DiagnosticsManager::start() {
    if (impl_->running.exchange(true)) return;   // already running
    impl_->thread = std::thread([this] { impl_->run(); });
}

void DiagnosticsManager::stop() {
    if (!impl_->running.exchange(false)) return;  // already stopped
    {
        std::lock_guard<std::mutex> lk(impl_->cv_mu);
        impl_->cv.notify_all();
    }
    if (impl_->thread.joinable())
        impl_->thread.join();
}

bool DiagnosticsManager::is_running() const noexcept {
    return impl_->running.load(std::memory_order_relaxed);
}

DiagnosticsSnapshot DiagnosticsManager::latest_snapshot() const {
    std::lock_guard<std::mutex> lk(impl_->snap_mu);
    return impl_->latest;
}

std::string DiagnosticsManager::to_json() const {
    return latest_snapshot().to_json();
}

THUNDERBIRD_ABI_NAMESPACE_END
} // namespace thunderbird
