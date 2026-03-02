// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Diagnostics system unit tests
// ─────────────────────────────────────────────────────────────────────────────
//
// Tests cover:
//   1. Default-constructed DiagnosticsManager is not running
//   2. start() / stop() lifecycle
//   3. Collector registration and execution
//   4. Snapshot contains registered metrics after a collection cycle
//   5. compute_rates() for Counter, Gauge, Flag
//   6. to_json() output validity (basic checks)
//   7. JSON escaping of special characters
//   8. Double start/stop idempotence
//
// ─────────────────────────────────────────────────────────────────────────────

#include "thunderbird/diagnostics.h"

#include <cstdio>
#include <cstdlib>
#include <thread>

namespace tbd = thunderbird;

#define VERIFY(expr) \
    do { if (!(expr)) { std::fprintf(stderr, "FAIL: %s @ %s:%d\n", #expr, __FILE__, __LINE__); std::abort(); } } while (0)

// ─── Tests ──────────────────────────────────────────────────────────────────

static void test_default_not_running() {
    std::printf("  default not running ... ");
    tbd::DiagnosticsManager mgr;
    VERIFY(!mgr.is_running());
    VERIFY(mgr.latest_snapshot().empty());
    std::printf("OK\n");
}

static void test_start_stop_lifecycle() {
    std::printf("  start/stop lifecycle ... ");
    tbd::DiagnosticsConfig cfg;
    cfg.interval = std::chrono::milliseconds(50);   // fast for tests
    tbd::DiagnosticsManager mgr(cfg);

    mgr.register_collector("test", []() -> tbd::MetricMap {
        return {{"counter_a", {tbd::MetricType::Counter, 42.0}}};
    });

    mgr.start();
    VERIFY(mgr.is_running());

    mgr.stop();
    VERIFY(!mgr.is_running());
    std::printf("OK\n");
}

static void test_collector_execution() {
    std::printf("  collector execution ... ");
    tbd::DiagnosticsConfig cfg;
    cfg.interval       = std::chrono::milliseconds(30);
    cfg.compute_rates  = false;
    tbd::DiagnosticsManager mgr(cfg);

    mgr.register_collector("sys", []() -> tbd::MetricMap {
        return {
            {"frames",  {tbd::MetricType::Counter, 100.0}},
            {"drift",   {tbd::MetricType::Gauge,   1.5}},
            {"healthy", {tbd::MetricType::Flag,     1.0}},
        };
    });

    mgr.start();
    // Wait for at least one collection cycle.
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    auto snap = mgr.latest_snapshot();
    mgr.stop();

    VERIFY(!snap.empty());
    VERIFY(snap.groups.count("sys") == 1);

    auto& m = snap.groups.at("sys");
    VERIFY(m.count("frames")  == 1);
    VERIFY(m.count("drift")   == 1);
    VERIFY(m.count("healthy") == 1);

    VERIFY(m.at("frames").type  == tbd::MetricType::Counter);
    VERIFY(m.at("frames").value == 100.0);
    VERIFY(m.at("drift").value  == 1.5);
    VERIFY(m.at("healthy").value == 1.0);
    std::printf("OK\n");
}

static void test_compute_rates() {
    std::printf("  compute_rates ... ");

    tbd::DiagnosticsSnapshot prev, curr;
    prev.timestamp = std::chrono::steady_clock::now();
    curr.timestamp = prev.timestamp + std::chrono::seconds(2);

    prev.groups["g"] = {
        {"cnt", {tbd::MetricType::Counter, 10.0}},
        {"gau", {tbd::MetricType::Gauge,   5.0}},
        {"flg", {tbd::MetricType::Flag,    0.0}},
    };
    curr.groups["g"] = {
        {"cnt", {tbd::MetricType::Counter, 20.0}},
        {"gau", {tbd::MetricType::Gauge,   7.0}},
        {"flg", {tbd::MetricType::Flag,    1.0}},
    };

    auto rates = tbd::DiagnosticsSnapshot::compute_rates(prev, curr);

    // Counter: (20 - 10) / 2 = 5.0
    VERIFY(rates.groups.count("g") == 1);
    auto& r = rates.groups.at("g");
    VERIFY(r.at("cnt").value == 5.0);

    // Gauge: taken as-is from current
    VERIFY(r.at("gau").value == 7.0);

    // Flag: taken as-is from current
    VERIFY(r.at("flg").value == 1.0);
    std::printf("OK\n");
}

static void test_to_json_basic() {
    std::printf("  to_json basic ... ");
    tbd::DiagnosticsSnapshot snap;
    snap.timestamp = std::chrono::steady_clock::now();
    snap.groups["sensor"] = {
        {"count", {tbd::MetricType::Counter, 42.0}},
        {"temp",  {tbd::MetricType::Gauge,   23.5}},
    };

    std::string json = snap.to_json();
    VERIFY(!json.empty());
    VERIFY(json.front() == '{');
    VERIFY(json.back()  == '}');
    // Must contain the group and metric names.
    VERIFY(json.find("\"sensor\"") != std::string::npos);
    VERIFY(json.find("\"count\"")  != std::string::npos);
    VERIFY(json.find("\"temp\"")   != std::string::npos);
    std::printf("OK\n");
}

static void test_json_escaping() {
    std::printf("  JSON escaping ... ");
    tbd::DiagnosticsSnapshot snap;
    snap.timestamp = std::chrono::steady_clock::now();
    // Group name with characters that must be escaped.
    snap.groups["a\"b\\c"] = {
        {"val\n1", {tbd::MetricType::Gauge, 1.0}},
    };

    std::string json = snap.to_json();
    // The double-quote in the group name must be escaped.
    VERIFY(json.find("a\\\"b\\\\c") != std::string::npos);
    // The newline in the metric name must be escaped.
    VERIFY(json.find("val\\n1") != std::string::npos);
    std::printf("OK\n");
}

static void test_empty_snapshot_json() {
    std::printf("  empty snapshot JSON ... ");
    tbd::DiagnosticsSnapshot snap;
    std::string json = snap.to_json();
    VERIFY(!json.empty());
    VERIFY(json.front() == '{');
    std::printf("OK\n");
}

static void test_double_start_stop() {
    std::printf("  double start/stop ... ");
    tbd::DiagnosticsConfig cfg;
    cfg.interval = std::chrono::milliseconds(50);
    tbd::DiagnosticsManager mgr(cfg);

    mgr.start();
    mgr.start();   // second start — should be no-op or safe
    VERIFY(mgr.is_running());

    mgr.stop();
    mgr.stop();    // second stop — should be safe
    VERIFY(!mgr.is_running());
    std::printf("OK\n");
}

// ─────────────────────────────────────────────────────────────────────────────

int main() {
    std::printf("test_diagnostics\n");

    test_default_not_running();
    test_start_stop_lifecycle();
    test_collector_execution();
    test_compute_rates();
    test_to_json_basic();
    test_json_escaping();
    test_empty_snapshot_json();
    test_double_start_stop();

    std::printf("All diagnostics tests passed.\n");
    return 0;
}
