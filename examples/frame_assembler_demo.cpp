// ─────────────────────────────────────────────────────────────────────────────
// Example — LidarFrameAssembler usage
// ─────────────────────────────────────────────────────────────────────────────
//
// Demonstrates:
//   1. Creating a LidarFrameAssembler in AzimuthWrap mode (spinning LiDAR)
//   2. Creating a LidarFrameAssembler in TimeBased mode (solid-state LiDAR)
//   3. Feeding per-packet LidarFrame data with azimuth ranges
//   4. Receiving full 360° sweeps via on_frame() callback
//   5. Inspecting FrameAssemblyMeta (coverage, drops, partial flag)
//   6. Reading AssemblerStats
//   7. Partial-frame timeout handling
//   8. Configuration tuning
//
// This example uses synthetic LidarFrame data — no real hardware required.
//
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/lidar_frame_assembler.h"

#include <cmath>
#include <cstdio>
#include <memory>

using namespace thunderbird;

// ── Helper: create a synthetic LidarFrame at a given azimuth ────────────────

static std::shared_ptr<LidarFrame> make_packet(
    uint32_t seq, float az_start, float az_end,
    int64_t hw_ns, int64_t host_ns, int num_points = 192)
{
    auto f = std::make_shared<LidarFrame>();
    f->timestamp       = Timestamp{hw_ns};
    f->host_timestamp  = Timestamp{host_ns};
    f->sequence_number = seq;

    // Generate points along the azimuth arc.
    float az_mid = (az_start + az_end) * 0.5f;
    float az_rad = az_mid * 3.14159265f / 180.0f;
    for (int i = 0; i < num_points; ++i) {
        float dist = 5.0f + 0.01f * i;
        float elev = -15.0f + (i % 16) * 2.0f;
        float elev_rad = elev * 3.14159265f / 180.0f;
        float cos_e = std::cos(elev_rad);
        LidarPoint pt;
        pt.x = dist * cos_e * std::sin(az_rad);
        pt.y = dist * cos_e * std::cos(az_rad);
        pt.z = dist * std::sin(elev_rad);
        pt.intensity = static_cast<float>(i % 256);
        pt.ring = static_cast<uint8_t>(i % 16);
        f->points.push_back(pt);
    }
    return f;
}

int main() {
    std::puts("=== LidarFrameAssembler Demo ===\n");

    // ═════════════════════════════════════════════════════════════════════════
    //  Part 1: AzimuthWrap mode — spinning LiDAR (e.g. VLP-16)
    // ═════════════════════════════════════════════════════════════════════════

    std::printf("--- Part 1: AzimuthWrap mode ---\n\n");

    FrameAssemblerConfig az_cfg;
    az_cfg.mode                     = CompletionMode::AzimuthWrap;
    az_cfg.expected_rate_hz         = 10.0;
    az_cfg.max_points_per_frame     = 50'000;
    az_cfg.min_points_to_emit       = 50;
    az_cfg.azimuth_wrap_threshold_deg = 180.0f;

    LidarFrameAssembler assembler(az_cfg);

    int frame_count = 0;
    assembler.on_frame(
        [&frame_count](std::shared_ptr<const odom::PointCloudFrame> cloud,
                       const FrameAssemblyMeta& meta) {
            ++frame_count;
            std::printf("  [Frame #%u] %zu points  az=[%.1f°, %.1f°]  "
                        "coverage=%.1f°  packets=%u  drops=%u  partial=%s\n",
                        meta.sequence,
                        cloud->points.size(),
                        meta.azimuth_start_deg,
                        meta.azimuth_end_deg,
                        meta.azimuth_coverage_deg,
                        meta.total_packets,
                        meta.dropped_packets,
                        meta.is_partial ? "yes" : "no");
        });

    // Simulate 2 full revolutions + start of a third.
    // Each revolution = 150 packets of ~2.4° each = 360°.
    const int packets_per_rev = 150;
    const float deg_per_pkt   = 360.0f / static_cast<float>(packets_per_rev);
    const int64_t pkt_dt_ns   = 666'667;      // ~100 ms / 150 packets
    int64_t hw_ns   = 1'000'000'000;
    int64_t host_ns = 1'000'000'000;

    for (int rev = 0; rev < 2; ++rev) {
        for (int p = 0; p < packets_per_rev; ++p) {
            float az_start = p * deg_per_pkt;
            float az_end   = az_start + deg_per_pkt;
            uint32_t seq = static_cast<uint32_t>(rev * packets_per_rev + p);
            auto pkt = make_packet(seq, az_start, az_end, hw_ns, host_ns, 192);
            assembler.feed(pkt, az_start, az_end);
            hw_ns   += pkt_dt_ns;
            host_ns += pkt_dt_ns;
        }
    }
    // Feed a few packets into the third revolution to trigger emission.
    for (int p = 0; p < 5; ++p) {
        float az_start = p * deg_per_pkt;
        float az_end   = az_start + deg_per_pkt;
        uint32_t seq = static_cast<uint32_t>(2 * packets_per_rev + p);
        auto pkt = make_packet(seq, az_start, az_end, hw_ns, host_ns, 192);
        assembler.feed(pkt, az_start, az_end);
        hw_ns   += pkt_dt_ns;
        host_ns += pkt_dt_ns;
    }

    std::printf("\nFrames emitted: %d\n", frame_count);

    // Print stats.
    auto st = assembler.stats();
    std::printf("\n--- AssemblerStats ---\n"
                "  frames_emitted:       %llu\n"
                "  partial_frames:       %llu\n"
                "  packets_ingested:     %llu\n"
                "  points_ingested:      %llu\n"
                "  runt_frames_discarded:%llu\n"
                "  avg_points/frame:     %.0f\n"
                "  avg_packets/frame:    %.0f\n"
                "  measured_rate_hz:     %.1f\n",
                static_cast<unsigned long long>(st.frames_emitted),
                static_cast<unsigned long long>(st.partial_frames),
                static_cast<unsigned long long>(st.packets_ingested),
                static_cast<unsigned long long>(st.points_ingested),
                static_cast<unsigned long long>(st.runt_frames_discarded),
                st.avg_points_per_frame,
                st.avg_packets_per_frame,
                st.measured_rate_hz);

    // ═════════════════════════════════════════════════════════════════════════
    //  Part 2: TimeBased mode — solid-state LiDAR
    // ═════════════════════════════════════════════════════════════════════════

    std::printf("\n--- Part 2: TimeBased mode ---\n\n");

    FrameAssemblerConfig time_cfg;
    time_cfg.mode             = CompletionMode::TimeBased;
    time_cfg.frame_period_ns  = 100'000'000;  // 100 ms
    time_cfg.min_points_to_emit = 50;

    LidarFrameAssembler time_asm(time_cfg);

    int time_frames = 0;
    time_asm.on_frame(
        [&time_frames](std::shared_ptr<const odom::PointCloudFrame> cloud,
                       const FrameAssemblyMeta& meta) {
            ++time_frames;
            std::printf("  [TimeBased Frame #%u] %zu points  "
                        "duration=%.1f ms  partial=%s\n",
                        meta.sequence,
                        cloud->points.size(),
                        meta.scan_duration_ns / 1e6,
                        meta.is_partial ? "yes" : "no");
        });

    // Feed 500 packets over 500 ms → expect 4 full frames (100 ms each)
    // plus one partial frame flushed by check_timeout().
    hw_ns   = 2'000'000'000;
    host_ns = 2'000'000'000;
    for (int i = 0; i < 500; ++i) {
        auto pkt = make_packet(static_cast<uint32_t>(i), 0, 0, hw_ns, host_ns, 100);
        time_asm.feed_timed(pkt);
        hw_ns   += 1'000'000;   // 1 ms between packets
        host_ns += 1'000'000;
    }
    // Advance time beyond partial_timeout to flush the trailing partial frame.
    // Default partial_timeout_mul = 1.5 × 100 ms period = 150 ms.
    int64_t flush_ns = host_ns + 250'000'000;  // +250 ms well past timeout
    time_asm.check_timeout(flush_ns);

    std::printf("\nTimeBased frames emitted: %d\n", time_frames);

    // ═════════════════════════════════════════════════════════════════════════
    //  Part 3: Partial-frame timeout
    // ═════════════════════════════════════════════════════════════════════════

    std::printf("\n--- Part 3: Partial-frame timeout ---\n\n");

    FrameAssemblerConfig timeout_cfg;
    timeout_cfg.mode               = CompletionMode::AzimuthWrap;
    timeout_cfg.expected_rate_hz   = 10.0;
    timeout_cfg.partial_timeout_mul = 2.0;  // 200 ms timeout
    timeout_cfg.min_points_to_emit = 10;

    LidarFrameAssembler timeout_asm(timeout_cfg);

    bool got_partial = false;
    timeout_asm.on_frame(
        [&got_partial](std::shared_ptr<const odom::PointCloudFrame> cloud,
                       const FrameAssemblyMeta& meta) {
            got_partial = meta.is_partial;
            std::printf("  [Timeout Frame] %zu points  partial=%s\n",
                        cloud->points.size(),
                        meta.is_partial ? "yes" : "no");
        });

    // Feed a few packets but never complete a revolution.
    for (int i = 0; i < 10; ++i) {
        float az = i * 5.0f;
        auto pkt = make_packet(static_cast<uint32_t>(i), az, az + 5.0f,
                               3'000'000'000LL + i * 1'000'000,
                               3'000'000'000LL + i * 1'000'000, 50);
        timeout_asm.feed(pkt, az, az + 5.0f);
    }

    // No frame emitted yet — now trigger timeout.
    int64_t timeout_now = 3'000'000'000LL + 300'000'000LL;  // 300 ms later
    timeout_asm.check_timeout(timeout_now);
    std::printf("Partial frame forced by timeout: %s\n",
                got_partial ? "yes" : "no");

    std::puts("\nDone.");
    return 0;
}
