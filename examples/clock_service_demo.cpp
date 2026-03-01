// ─────────────────────────────────────────────────────────────────────────────
// Example — ClockService usage
// ─────────────────────────────────────────────────────────────────────────────
//
// Demonstrates:
//   1. Creating a ClockService with Drone and Car profiles
//   2. Feeding (hardware, host) timestamp pairs via observe()
//   3. Converting timestamps between clock domains with hw_to_host()
//   4. Getting unified timestamps via unified_timestamp()
//   5. Subscribing to clock events (Calibrated, DriftWarning, TimeJump)
//   6. Reading diagnostic snapshots
//   7. Handling a simulated time jump
//   8. PPS anchoring (Car profile)
//
// This example uses synthetic timestamps — no real hardware required.
//
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/clock_service.h"

#include <chrono>
#include <cstdio>

static const char* clock_event_name(thunderbird::ClockEvent e) {
    switch (e) {
        case thunderbird::ClockEvent::Calibrated:   return "Calibrated";
        case thunderbird::ClockEvent::DriftWarning: return "DriftWarning";
        case thunderbird::ClockEvent::TimeJump:     return "TimeJump";
        case thunderbird::ClockEvent::PpsLocked:    return "PpsLocked";
        case thunderbird::ClockEvent::PpsLost:      return "PpsLost";
        case thunderbird::ClockEvent::ModelReset:   return "ModelReset";
    }
    return "Unknown";
}

int main() {
    using namespace thunderbird;
    using namespace std::chrono;

    std::puts("=== ClockService Demo ===\n");

    // ── 1. Create a ClockService (Drone profile — free-running crystal) ─────
    ClockServiceConfig drone_cfg;
    drone_cfg.profile      = ClockProfile::Drone;
    drone_cfg.ols_window   = 50;       // Smaller window for demo
    drone_cfg.jump_threshold_ns     = 50'000'000;   // 50 ms
    drone_cfg.jump_confirm_count    = 3;
    drone_cfg.drift_warn_threshold_ns_per_sec = 500'000.0;  // 0.5 ms/s

    ClockService clock(drone_cfg);

    // ── 2. Register event callback ──────────────────────────────────────────
    clock.on_event([](ClockEvent event, const ClockDiagnostics& diag) {
        std::printf("  [ClockEvent] %-14s  offset=%.0f ns  drift=%.1f ns/s"
                    "  obs=%llu  outliers=%llu\n",
                    clock_event_name(event),
                    diag.offset_ns,
                    diag.drift_ns_per_sec,
                    static_cast<unsigned long long>(diag.observations),
                    static_cast<unsigned long long>(diag.outliers_rejected));
    });

    // ── 3. Feed simulated (hw, host) pairs ──────────────────────────────────
    //
    // Simulate a device clock that runs 10 ppm faster than host clock.
    // hw_ns ≈ host_ns × 1.00001 + offset
    //
    std::printf("--- Feeding 20 observations (10 ppm drift) ---\n");

    const int64_t base_host = 1'000'000'000'000LL;  // 1000 seconds
    const int64_t base_hw   = 1'000'010'000'000LL;  // 10 ms ahead (offset)
    const double  drift_ppm = 10.0;
    const int64_t step_ns   = 10'000'000;            // 10 ms between samples

    for (int i = 0; i < 20; ++i) {
        int64_t host_ns = base_host + i * step_ns;
        // hw = host * (1 + drift_ppm * 1e-6) + offset
        int64_t hw_ns = base_hw +
            static_cast<int64_t>(i * step_ns * (1.0 + drift_ppm * 1e-6));
        clock.observe(hw_ns, host_ns);
    }

    // ── 4. Query calibration state ──────────────────────────────────────────
    std::printf("\nCalibrated: %s\n", clock.is_calibrated() ? "yes" : "no");
    std::printf("Drift:      %.3f ns/s (%.4f ppm)\n",
                clock.drift_ns_per_sec(),
                clock.drift_ns_per_sec() / 1000.0);

    // ── 5. Convert a hardware timestamp to host domain ──────────────────────
    int64_t test_hw = base_hw + 500 * step_ns;
    int64_t converted = clock.hw_to_host(test_hw);
    std::printf("\nhw_to_host(%lld) = %lld\n",
                static_cast<long long>(test_hw),
                static_cast<long long>(converted));

    // Inverse: host_to_hw
    int64_t roundtrip = clock.host_to_hw(converted);
    std::printf("host_to_hw(%lld) = %lld  (diff=%lld ns)\n",
                static_cast<long long>(converted),
                static_cast<long long>(roundtrip),
                static_cast<long long>(roundtrip - test_hw));

    // ── 6. unified_timestamp() — picks best available ───────────────────────
    int64_t host_now = base_host + 500 * step_ns;
    int64_t unified = clock.unified_timestamp(test_hw, host_now);
    std::printf("unified_timestamp(hw, host) = %lld  "
                "(uses calibrated model)\n",
                static_cast<long long>(unified));

    // ── 7. Read full diagnostics ────────────────────────────────────────────
    auto diag = clock.diagnostics();
    std::printf("\n--- Diagnostics ---\n"
                "  offset_ns:         %.0f\n"
                "  drift_ns_per_sec:  %.3f\n"
                "  offset_stddev_ns:  %.1f\n"
                "  observations:      %llu\n"
                "  jumps_detected:    %llu\n"
                "  outliers_rejected: %llu\n"
                "  calibrated:        %s\n"
                "  pps_locked:        %s\n",
                diag.offset_ns,
                diag.drift_ns_per_sec,
                diag.offset_stddev_ns,
                static_cast<unsigned long long>(diag.observations),
                static_cast<unsigned long long>(diag.jumps_detected),
                static_cast<unsigned long long>(diag.outliers_rejected),
                diag.calibrated ? "yes" : "no",
                diag.pps_locked ? "yes" : "no");

    // ── 8. Simulate a time jump ─────────────────────────────────────────────
    //
    // After N normal observations, suddenly shift hw timestamps by +200 ms.
    // The ClockService should detect the jump and re-seed its model.
    //
    std::printf("\n--- Simulating time jump (+200 ms in hw clock) ---\n");

    int64_t jump_offset = 200'000'000;  // 200 ms
    for (int i = 20; i < 30; ++i) {
        int64_t host_ns = base_host + i * step_ns;
        int64_t hw_ns = base_hw + jump_offset +
            static_cast<int64_t>(i * step_ns * (1.0 + drift_ppm * 1e-6));
        clock.observe(hw_ns, host_ns);
    }

    diag = clock.diagnostics();
    std::printf("After jump: jumps_detected=%llu  outliers_rejected=%llu\n",
                static_cast<unsigned long long>(diag.jumps_detected),
                static_cast<unsigned long long>(diag.outliers_rejected));

    // ── 9. PPS anchoring (Car profile) ──────────────────────────────────────
    std::printf("\n--- PPS Anchoring (Car profile) ---\n");

    ClockServiceConfig car_cfg;
    car_cfg.profile    = ClockProfile::Car;
    car_cfg.enable_pps = true;
    car_cfg.pps_interval_ns  = 1'000'000'000;  // 1 second
    car_cfg.pps_tolerance_ns = 10'000'000;      // 10 ms

    ClockService car_clock(car_cfg);

    car_clock.on_event([](ClockEvent event, const ClockDiagnostics& diag) {
        std::printf("  [Car ClockEvent] %-14s  pps_locked=%s\n",
                    clock_event_name(event),
                    diag.pps_locked ? "yes" : "no");
    });

    // Feed enough observations for calibration.
    for (int i = 0; i < 10; ++i) {
        int64_t host_ns = 2'000'000'000'000LL + i * 100'000'000LL;
        int64_t hw_ns   = host_ns + 5'000;  // 5 μs offset
        car_clock.observe(hw_ns, host_ns);
    }

    // Feed 4 PPS pulses at ~1 second intervals to achieve lock.
    for (int i = 0; i < 4; ++i) {
        int64_t pps_host = 2'000'000'000'000LL + i * 1'000'000'000LL;
        int64_t pps_hw   = pps_host + 5'000;
        car_clock.observe_pps(pps_host, pps_hw);
    }

    auto car_diag = car_clock.diagnostics();
    std::printf("Car clock: calibrated=%s  pps_locked=%s\n",
                car_diag.calibrated ? "yes" : "no",
                car_diag.pps_locked ? "yes" : "no");

    // ── 10. Reset ───────────────────────────────────────────────────────────
    clock.reset();
    std::printf("\nAfter reset: calibrated=%s\n",
                clock.is_calibrated() ? "yes" : "no");

    std::puts("\nDone.");
    return 0;
}
