// ─────────────────────────────────────────────────────────────────────────────
// Example — Third-party LiDAR end-to-end integration
// ─────────────────────────────────────────────────────────────────────────────
//
// Demonstrates the core Phase 2 decoding pipeline for a non-Thunderbird LiDAR
// (Velodyne VLP-16) using three subsystems together:
//
//   1. DecoderFactory — create a VLP-16 decoder by model name
//   2. ClockService — synchronize VLP-16 GPS timestamps to host clock
//   3. LidarFrameAssembler — accumulate per-packet data into full sweeps
//
// The demo feeds synthetic VLP-16 packets directly into the decoder to
// show the decode → clock-sync → frame-assembly data flow.  In production,
// a ConnectionManager with raw_streaming_mode would drive the I/O loop
// and a DeviceHealthMonitor would track packet rates (see health_monitor_demo
// and comm_layer_demo for those APIs).
//
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/thunderbird.h"
#include "thunderbird/decoders/decoder_factory.h"
#include "thunderbird/decoders/velodyne_vlp16.h"
#include "thunderbird/clock_service.h"
#include "thunderbird/lidar_frame_assembler.h"

#include <cstdio>
#include <cstring>

using namespace thunderbird;

// ── Helper: build a synthetic VLP-16 UDP packet (1206 bytes) ────────────────

static void build_vlp16_packet(uint8_t* buf, uint32_t gps_us, float base_azimuth) {
    std::memset(buf, 0, vlp16::kPacketSize);

    for (size_t b = 0; b < vlp16::kNumFiringBlocks; ++b) {
        size_t offset = b * sizeof(vlp16::FiringBlock);

        // Flag = 0xFFEE
        uint16_t flag = vlp16::kFiringBlockFlag;
        std::memcpy(buf + offset, &flag, sizeof(flag));

        // Azimuth: base + block * step
        float az_deg = base_azimuth + static_cast<float>(b) * 0.2f;
        if (az_deg >= 360.0f) az_deg -= 360.0f;
        uint16_t raw_az = static_cast<uint16_t>(az_deg / vlp16::kAzimuthUnit);
        std::memcpy(buf + offset + 2, &raw_az, sizeof(raw_az));

        // Fill 16 channels with synthetic distances.
        for (size_t ch = 0; ch < vlp16::kSingleReturnChans; ++ch) {
            size_t ch_off = offset + 4 + ch * 3;
            uint16_t dist = static_cast<uint16_t>(500 + b * 10 + ch * 5);
            buf[ch_off]     = static_cast<uint8_t>(dist & 0xFF);
            buf[ch_off + 1] = static_cast<uint8_t>(dist >> 8);
            buf[ch_off + 2] = static_cast<uint8_t>(128);  // reflectivity
        }
    }

    // GPS timestamp (offset 1200)
    std::memcpy(buf + 1200, &gps_us, sizeof(gps_us));
    // Return mode: strongest
    buf[1204] = 0x37;
    // Product ID: VLP-16
    buf[1205] = 0x22;
}

// ─────────────────────────────────────────────────────────────────────────────

int main() {
    std::puts("=== Third-Party LiDAR End-to-End Demo ===\n");

    // ── 1. Create decoder via factory ───────────────────────────────────────
    std::printf("--- 1. DecoderFactory ---\n");

    auto decoder = DecoderFactory::create("vlp16");
    if (!decoder) {
        std::fprintf(stderr, "ERROR: unknown decoder model\n");
        return 1;
    }
    std::printf("Created decoder: %s\n\n", decoder->decoder_name());

    // Keep a typed pointer for VLP-16–specific API (azimuth accessors).
    auto* vlp16_dec = dynamic_cast<VelodyneVlp16Decoder*>(decoder.get());

    // ── 2. Set up ClockService ──────────────────────────────────────────────
    std::printf("--- 2. ClockService ---\n");

    ClockServiceConfig clock_cfg;
    clock_cfg.profile    = ClockProfile::Drone;
    clock_cfg.ols_window = 50;
    ClockService clock(clock_cfg);

    clock.on_event([](ClockEvent event, const ClockDiagnostics& /*diag*/) {
        const char* name = "Unknown";
        switch (event) {
            case ClockEvent::Calibrated:   name = "Calibrated";   break;
            case ClockEvent::DriftWarning: name = "DriftWarning"; break;
            case ClockEvent::TimeJump:     name = "TimeJump";     break;
            case ClockEvent::PpsLocked:    name = "PpsLocked";    break;
            case ClockEvent::PpsLost:      name = "PpsLost";      break;
            case ClockEvent::ModelReset:   name = "ModelReset";   break;
        }
        std::printf("  [Clock] %s\n", name);
    });
    std::printf("ClockService created (Drone profile)\n\n");

    // ── 3. Set up LidarFrameAssembler ───────────────────────────────────────
    std::printf("--- 3. LidarFrameAssembler ---\n");

    FrameAssemblerConfig asm_cfg;
    asm_cfg.mode                     = CompletionMode::AzimuthWrap;
    asm_cfg.expected_rate_hz         = 10.0;
    asm_cfg.min_points_to_emit       = 100;
    asm_cfg.azimuth_wrap_threshold_deg = 180.0f;

    LidarFrameAssembler assembler(asm_cfg);

    int sweep_count = 0;
    assembler.on_frame(
        [&sweep_count, &clock](
            std::shared_ptr<const odom::PointCloudFrame> cloud,
            const FrameAssemblyMeta& meta) {
            ++sweep_count;
            // Use ClockService to convert scan timestamp.
            int64_t unified_ts = clock.unified_timestamp(
                meta.scan_start_ns, meta.host_arrival_ns);
            std::printf("  [Sweep #%u] %zu pts  az=[%.1f°,%.1f°]  "
                        "coverage=%.0f°  drops=%u  unified_ts=%lld\n",
                        meta.sequence,
                        cloud->points.size(),
                        meta.azimuth_start_deg,
                        meta.azimuth_end_deg,
                        meta.azimuth_coverage_deg,
                        meta.dropped_packets,
                        static_cast<long long>(unified_ts));
        });
    std::printf("Assembler created (AzimuthWrap mode)\n\n");

    // ── 4. Wire decoder → clock + assembler ─────────────────────────────────
    //
    // When the decoder emits a LidarFrame, we:
    //   a) Feed the clock service with (hw_ts, host_ts) observation
    //   b) Feed the frame assembler with azimuth information
    //
    decoder->on_lidar([&](std::shared_ptr<const LidarFrame> frame) {
        // Clock observation.
        clock.observe(frame->timestamp.nanoseconds,
                      frame->host_timestamp.nanoseconds);

        // Frame assembler.
        float az_start = vlp16_dec ? vlp16_dec->last_azimuth_start() : 0.0f;
        float az_end   = vlp16_dec ? vlp16_dec->last_azimuth_end()   : 0.0f;
        assembler.feed(frame, az_start, az_end);
    });

    // ── 5. Simulate VLP-16 packets ──────────────────────────────────────────
    std::printf("--- 4. Feeding synthetic VLP-16 packets ---\n");

    // Simulate 3 full revolutions.
    // VLP-16: 754 packets/sec at 10 Hz rotation = ~75 packets/revolution.
    const int pkts_per_rev = 75;
    const float deg_per_pkt = 360.0f / static_cast<float>(pkts_per_rev);
    const int64_t pkt_interval_us = 1333;  // ~1.333 ms between packets
    uint32_t gps_us = 1'000'000;           // 1 second into the hour

    for (int rev = 0; rev < 3; ++rev) {
        for (int p = 0; p < pkts_per_rev; ++p) {
            uint8_t pkt[vlp16::kPacketSize];
            float azimuth = p * deg_per_pkt;
            build_vlp16_packet(pkt, gps_us, azimuth);

            decoder->feed(pkt, vlp16::kPacketSize);
            gps_us += static_cast<uint32_t>(pkt_interval_us);
        }
    }
    // Feed a few more packets to trigger the 3rd frame emission.
    for (int p = 0; p < 5; ++p) {
        uint8_t pkt[vlp16::kPacketSize];
        build_vlp16_packet(pkt, gps_us, p * deg_per_pkt);
        decoder->feed(pkt, vlp16::kPacketSize);
        gps_us += static_cast<uint32_t>(pkt_interval_us);
    }

    std::printf("\nTotal sweeps assembled: %d\n", sweep_count);

    // ── 6. Print decoder stats ──────────────────────────────────────────────
    auto ds = decoder->stats();
    std::printf("\n--- Decoder Stats ---\n"
                "  packets_parsed:   %llu\n"
                "  checksum_errors:  %llu\n"
                "  malformed_count:  %llu\n"
                "  bytes_processed:  %llu\n",
                static_cast<unsigned long long>(ds.packets_parsed),
                static_cast<unsigned long long>(ds.checksum_errors),
                static_cast<unsigned long long>(ds.malformed_count),
                static_cast<unsigned long long>(ds.bytes_processed));

    // ── 7. Print clock diagnostics ──────────────────────────────────────────
    auto cd = clock.diagnostics();
    std::printf("\n--- Clock Diagnostics ---\n"
                "  calibrated:    %s\n"
                "  observations:  %llu\n"
                "  drift_ns/s:    %.3f\n"
                "  offset_ns:     %.0f\n",
                cd.calibrated ? "yes" : "no",
                static_cast<unsigned long long>(cd.observations),
                cd.drift_ns_per_sec,
                cd.offset_ns);

    // ── 8. Print assembler stats ────────────────────────────────────────────
    auto as = assembler.stats();
    std::printf("\n--- Assembler Stats ---\n"
                "  frames_emitted:   %llu\n"
                "  packets_ingested: %llu\n"
                "  points_ingested:  %llu\n"
                "  avg_pts/frame:    %.0f\n",
                static_cast<unsigned long long>(as.frames_emitted),
                static_cast<unsigned long long>(as.packets_ingested),
                static_cast<unsigned long long>(as.points_ingested),
                as.avg_points_per_frame);

    std::puts("\nDone.");
    return 0;
}
