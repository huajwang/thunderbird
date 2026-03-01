// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Unified Diagnostics System
// ─────────────────────────────────────────────────────────────────────────────
//
// Thread-safe diagnostics aggregator that collects metrics from all SDK
// subsystems (time-sync, health monitor, clock service, frame assembler,
// perception engine, recorder, decoder) via registered collector callbacks.
//
// Zero hot-path overhead: collectors run on a background thread at a
// configurable interval (default 1 Hz).  Each collector reads the
// existing stats() / snapshot() methods on the subsystem it monitors.
//
// Usage:
//   DiagnosticsManager diag;
//   diag.register_collector("time_sync", [&dm]() {
//       auto s = dm.time_sync().stats();
//       return DiagnosticsSnapshot::MetricMap{
//           {"sync.frames_produced", {MetricType::Counter, s.frames_produced}},
//           ...
//       };
//   });
//   diag.start();                        // spawns collector thread
//   auto snap = diag.latest_snapshot();  // latest metrics snapshot
//   auto json = diag.to_json();          // JSON serialization
//   diag.stop();
//
// Lifecycle helpers in DeviceManager:
//   auto snap = dev.diagnostics_snapshot();
//   auto json = dev.diagnostics_json();
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/export.h"
#include "thunderbird/abi.h"

#include <cstdint>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace thunderbird {
THUNDERBIRD_ABI_NAMESPACE_BEGIN

// ═════════════════════════════════════════════════════════════════════════════
//  Metric types
// ═════════════════════════════════════════════════════════════════════════════

/// Classification of a collected metric.
enum class MetricType : uint8_t {
    Counter = 0,   ///< Monotonically-increasing count (e.g. frames_produced)
    Gauge   = 1,   ///< Instantaneous value (e.g. drift_ns_per_sec)
    Flag    = 2,   ///< Boolean state (0.0 = false, non-zero = true)
};

/// A single metric value with its type tag.
struct MetricValue {
    MetricType type{MetricType::Gauge};
    double     value{0.0};
};

/// Map of metric name → value.
using MetricMap = std::unordered_map<std::string, MetricValue>;

// ═════════════════════════════════════════════════════════════════════════════
//  DiagnosticsSnapshot
// ═════════════════════════════════════════════════════════════════════════════

/// Point-in-time snapshot of all registered metrics.
struct DiagnosticsSnapshot {
    /// Steady-clock timestamp of when this snapshot was collected.
    std::chrono::steady_clock::time_point timestamp{};

    /// All metrics, grouped by collector name.
    /// Key: collector name (e.g. "time_sync"), Value: metric map.
    std::unordered_map<std::string, MetricMap> groups;

    /// Compute approximate rate (delta / elapsed) between two snapshots.
    /// Only applies to Counter-type metrics; Gauge and Flag are taken
    /// from `current`.
    static DiagnosticsSnapshot compute_rates(
        const DiagnosticsSnapshot& previous,
        const DiagnosticsSnapshot& current);

    /// Serialize this snapshot to a JSON string (compact format).
    [[nodiscard]] std::string to_json() const;

    /// Returns true if the snapshot contains any metrics.
    [[nodiscard]] bool empty() const noexcept { return groups.empty(); }
};

// ═════════════════════════════════════════════════════════════════════════════
//  DiagnosticsConfig
// ═════════════════════════════════════════════════════════════════════════════

/// Configuration for the diagnostics collector thread.
struct DiagnosticsConfig {
    /// Collection interval (default 1 Hz).
    std::chrono::milliseconds interval{1000};

    /// Whether to compute per-second rates for Counter metrics.
    bool compute_rates{true};
};

// ═════════════════════════════════════════════════════════════════════════════
//  Collector callback type
// ═════════════════════════════════════════════════════════════════════════════

/// A collector function returns a MetricMap of current metric values.
/// Called from the diagnostics thread at the configured interval.
/// Must be thread-safe and non-blocking.
using CollectorFn = std::function<MetricMap()>;

// ═════════════════════════════════════════════════════════════════════════════
//  DiagnosticsManager
// ═════════════════════════════════════════════════════════════════════════════

/// Central diagnostics aggregator.  Owns a background thread that
/// periodically invokes registered collectors and merges results into
/// a snapshot accessible from any thread.
class THUNDERBIRD_API DiagnosticsManager {
public:
    explicit DiagnosticsManager(DiagnosticsConfig config = {});
    ~DiagnosticsManager();

    // Non-copyable, non-movable (owns thread + PImpl).
    DiagnosticsManager(const DiagnosticsManager&)            = delete;
    DiagnosticsManager& operator=(const DiagnosticsManager&) = delete;
    DiagnosticsManager(DiagnosticsManager&&)                 = delete;
    DiagnosticsManager& operator=(DiagnosticsManager&&)      = delete;

    // ── Collector registration ──────────────────────────────────────────

    /// Register a named collector.  Must be called before start().
    /// @param name   Group label (e.g. "time_sync", "health", "assembler").
    /// @param fn     Callback returning a MetricMap.  Must be thread-safe.
    void register_collector(const std::string& name, CollectorFn fn);

    // ── Lifecycle ───────────────────────────────────────────────────────

    /// Spawn the collector thread.  Collectors begin executing after this.
    void start();

    /// Signal the collector thread to stop and join it.
    void stop();

    /// Whether the collector thread is running.
    [[nodiscard]] bool is_running() const noexcept;

    // ── Snapshot access ─────────────────────────────────────────────────

    /// Returns the latest snapshot (mutex-protected copy).
    /// Returns an empty snapshot if no collection cycle has completed yet.
    [[nodiscard]] DiagnosticsSnapshot latest_snapshot() const;

    /// Convenience: latest snapshot serialized as compact JSON.
    [[nodiscard]] std::string to_json() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

THUNDERBIRD_ABI_NAMESPACE_END
} // namespace thunderbird
