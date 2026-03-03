// ─────────────────────────────────────────────────────────────────────────────
// Test — Phase 2: VLP-16 decoder, DecoderFactory, SequenceTracker,
//                 ClockService, LidarFrameAssembler,
//                 ClockService↔SyncEngine integration
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/sequence_tracker.h"
#include "thunderbird/packet_decoder.h"
#include "thunderbird/decoders/velodyne_vlp16.h"
#include "thunderbird/decoders/decoder_factory.h"
#include "thunderbird/clock_service.h"
#include "thunderbird/lidar_frame_assembler.h"
#include "thunderbird/packet_parser.h"
#include "thunderbird/sync_engine.h"
#include "thunderbird/time_sync.h"
#include "thunderbird/types.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <thread>
#include <atomic>
#include <chrono>
#include <vector>

using namespace thunderbird;

// ═════════════════════════════════════════════════════════════════════════════
//  Test infrastructure
// ═════════════════════════════════════════════════════════════════════════════

static int g_tests_run    = 0;
static int g_tests_passed = 0;

#define TEST_CASE(name)                                                   \
    do {                                                                  \
        ++g_tests_run;                                                    \
        std::printf("  %-56s ", name);                                    \
    } while (0)

#define PASS()                                                            \
    do {                                                                  \
        ++g_tests_passed;                                                 \
        std::printf("[PASS]\n");                                          \
    } while (0)

#define FAIL(msg)                                                         \
    do {                                                                  \
        std::printf("[FAIL] %s (line %d)\n", msg, __LINE__);             \
        return;                                                           \
    } while (0)

#define ASSERT_EQ(a, b)                                                   \
    do { if ((a) != (b)) FAIL(#a " != " #b); } while (0)

#define ASSERT_NE(a, b)                                                   \
    do { if ((a) == (b)) FAIL(#a " == " #b); } while (0)

#define ASSERT_TRUE(expr)                                                 \
    do { if (!(expr)) FAIL(#expr " is false"); } while (0)

#define ASSERT_FALSE(expr)                                                \
    do { if ((expr)) FAIL(#expr " is true"); } while (0)

#define ASSERT_NEAR(a, b, tol)                                            \
    do {                                                                  \
        if (std::abs(static_cast<double>(a) - static_cast<double>(b))     \
            > (tol))                                                      \
            FAIL(#a " not near " #b);                                     \
    } while (0)

// ═════════════════════════════════════════════════════════════════════════════
//  SequenceTracker tests
// ═════════════════════════════════════════════════════════════════════════════

void test_sequence_tracker_sequential() {
    TEST_CASE("SequenceTracker — sequential (no drops)");
    SequenceTracker st;
    ASSERT_EQ(st.update(1), 0u);
    ASSERT_EQ(st.update(2), 0u);
    ASSERT_EQ(st.update(3), 0u);
    ASSERT_EQ(st.update(4), 0u);
    ASSERT_EQ(st.total_dropped(), 0u);
    PASS();
}

void test_sequence_tracker_gap() {
    TEST_CASE("SequenceTracker — gap detection");
    SequenceTracker st;
    st.update(1);
    st.update(2);
    uint32_t drops = st.update(5);  // skipped 3 and 4
    ASSERT_EQ(drops, 2u);
    ASSERT_EQ(st.total_dropped(), 2u);
    PASS();
}

void test_sequence_tracker_wrap() {
    TEST_CASE("SequenceTracker — uint32 wrap-around");
    SequenceTracker st;
    st.update(0xFFFFFFFE);
    st.update(0xFFFFFFFF);
    uint32_t drops = st.update(0);  // wrap: 0xFFFFFFFF → 0
    ASSERT_EQ(drops, 0u);
    PASS();
}

void test_sequence_tracker_large_gap_reset() {
    TEST_CASE("SequenceTracker — large gap treated as reset");
    SequenceTracker st;
    st.update(10);
    // Gap > kMaxReasonableGap → treated as device restart, not drops
    uint32_t drops = st.update(2000);
    ASSERT_EQ(drops, 0u);
    PASS();
}

void test_sequence_tracker_reset() {
    TEST_CASE("SequenceTracker — reset");
    SequenceTracker st;
    st.update(1);
    st.update(5);  // 3 dropped
    st.reset();
    ASSERT_EQ(st.total_dropped(), 0u);
    ASSERT_EQ(st.update(100), 0u);  // first after reset → no drop
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════════
//  VLP-16 decoder tests
// ═════════════════════════════════════════════════════════════════════════════

/// Build a synthetic VLP-16 data packet (1206 bytes).
static std::vector<uint8_t> build_vlp16_packet(
    float azimuth_deg = 45.0f,
    uint32_t gps_us = 100000,
    uint16_t distance = 500  // 1.0 m
) {
    std::vector<uint8_t> pkt(vlp16::kPacketSize, 0);

    for (size_t b = 0; b < vlp16::kNumFiringBlocks; ++b) {
        size_t off = b * 100;
        // Flag: 0xFFEE (little-endian)
        pkt[off + 0] = 0xEE;
        pkt[off + 1] = 0xFF;

        // Azimuth: increment per block
        float az = azimuth_deg + static_cast<float>(b) * 0.4f;
        uint16_t raw_az = static_cast<uint16_t>(az / vlp16::kAzimuthUnit);
        std::memcpy(&pkt[off + 2], &raw_az, 2);

        // Fill first 16 channels with valid distances
        for (size_t ch = 0; ch < vlp16::kSingleReturnChans; ++ch) {
            size_t ch_off = off + 4 + ch * 3;
            std::memcpy(&pkt[ch_off], &distance, 2);
            pkt[ch_off + 2] = 128;  // reflectivity
        }
    }

    // GPS timestamp @ offset 1200
    std::memcpy(&pkt[1200], &gps_us, 4);

    // Return mode and product ID
    pkt[1204] = 0x37;  // strongest
    pkt[1205] = 0x22;  // VLP-16

    return pkt;
}

void test_vlp16_valid_packet() {
    TEST_CASE("VLP-16 — valid packet produces points");
    VelodyneVlp16Decoder decoder;

    int lidar_count = 0;
    decoder.on_lidar([&](std::shared_ptr<const LidarFrame> f) {
        lidar_count++;
        ASSERT_TRUE(f->points.size() > 0);
        ASSERT_TRUE(f->points.size() <= 192);
    });

    auto pkt = build_vlp16_packet(90.0f, 200000, 1000);
    decoder.feed(pkt.data(), pkt.size());

    ASSERT_EQ(lidar_count, 1);
    ASSERT_EQ(decoder.stats().packets_parsed, 1u);
    ASSERT_EQ(decoder.stats().malformed_count, 0u);
    PASS();
}

void test_vlp16_wrong_size() {
    TEST_CASE("VLP-16 — wrong size is malformed");
    VelodyneVlp16Decoder decoder;

    int lidar_count = 0;
    decoder.on_lidar([&](auto) { lidar_count++; });

    uint8_t buf[100] = {};
    decoder.feed(buf, sizeof(buf));

    ASSERT_EQ(lidar_count, 0);
    ASSERT_EQ(decoder.stats().malformed_count, 1u);
    PASS();
}

void test_vlp16_bad_flag() {
    TEST_CASE("VLP-16 — bad block flag is malformed");
    VelodyneVlp16Decoder decoder;

    auto pkt = build_vlp16_packet();
    // Corrupt the first block flag
    pkt[0] = 0x00;
    pkt[1] = 0x00;

    int lidar_count = 0;
    decoder.on_lidar([&](auto) { lidar_count++; });
    decoder.feed(pkt.data(), pkt.size());

    ASSERT_EQ(lidar_count, 0);
    ASSERT_EQ(decoder.stats().malformed_count, 1u);
    PASS();
}

void test_vlp16_xyz_range() {
    TEST_CASE("VLP-16 — XYZ values in expected range");
    VelodyneVlp16Decoder decoder;

    std::shared_ptr<const LidarFrame> captured;
    decoder.on_lidar([&](auto f) { captured = f; });

    // 500 distance units = 1.0 m
    auto pkt = build_vlp16_packet(0.0f, 100000, 500);
    decoder.feed(pkt.data(), pkt.size());

    ASSERT_TRUE(captured != nullptr);
    for (const auto& pt : captured->points) {
        float dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        ASSERT_NEAR(dist, 1.0f, 0.01f);
    }
    PASS();
}

void test_vlp16_azimuth_tracking() {
    TEST_CASE("VLP-16 — azimuth start/end tracking");
    VelodyneVlp16Decoder decoder;

    decoder.on_lidar([](auto) {});  // need a callback to parse

    auto pkt = build_vlp16_packet(90.0f, 100000, 500);
    decoder.feed(pkt.data(), pkt.size());

    ASSERT_NEAR(decoder.last_azimuth_start(), 90.0f, 1.0f);
    ASSERT_TRUE(decoder.last_azimuth_end() > decoder.last_azimuth_start());
    PASS();
}

void test_vlp16_reset() {
    TEST_CASE("VLP-16 — reset clears stats");
    VelodyneVlp16Decoder decoder;
    decoder.on_lidar([](auto) {});

    auto pkt = build_vlp16_packet();
    decoder.feed(pkt.data(), pkt.size());
    ASSERT_EQ(decoder.stats().packets_parsed, 1u);

    decoder.reset();
    ASSERT_EQ(decoder.stats().packets_parsed, 0u);
    ASSERT_EQ(decoder.stats().bytes_processed, 0u);
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════════
//  DecoderFactory tests
// ═════════════════════════════════════════════════════════════════════════════

void test_factory_native() {
    TEST_CASE("DecoderFactory — 'thunderbird' creates PacketParser");
    auto d = DecoderFactory::create("thunderbird");
    ASSERT_TRUE(d != nullptr);
    ASSERT_TRUE(dynamic_cast<PacketParser*>(d.get()) != nullptr);
    PASS();
}

void test_factory_vlp16() {
    TEST_CASE("DecoderFactory — 'vlp16' creates VelodyneVlp16Decoder");
    auto d = DecoderFactory::create("vlp16");
    ASSERT_TRUE(d != nullptr);
    ASSERT_TRUE(dynamic_cast<VelodyneVlp16Decoder*>(d.get()) != nullptr);
    PASS();
}

void test_factory_case_insensitive() {
    TEST_CASE("DecoderFactory — case-insensitive matching");
    auto d = DecoderFactory::create("VLP16");
    ASSERT_TRUE(d != nullptr);
    PASS();
}

void test_factory_unknown() {
    TEST_CASE("DecoderFactory — unknown model returns nullptr");
    auto d = DecoderFactory::create("nonexistent_sensor");
    ASSERT_TRUE(d == nullptr);
    PASS();
}

void test_factory_empty() {
    TEST_CASE("DecoderFactory — empty string creates native");
    auto d = DecoderFactory::create("");
    ASSERT_TRUE(d != nullptr);
    ASSERT_TRUE(dynamic_cast<PacketParser*>(d.get()) != nullptr);
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════════
//  ClockService tests
// ═════════════════════════════════════════════════════════════════════════════

void test_clock_uncalibrated() {
    TEST_CASE("ClockService — uncalibrated returns hw_ns identity");
    ClockService clk;
    // Before any observations, hw_to_host should return input unchanged.
    ASSERT_EQ(clk.hw_to_host(1000000), 1000000);
    ASSERT_FALSE(clk.is_calibrated());
    PASS();
}

void test_clock_calibration() {
    TEST_CASE("ClockService — calibrates after 3 observations");
    ClockServiceConfig cfg;
    cfg.ols_window = 50;
    ClockService clk(cfg);

    // Feed observations with a fixed offset of 5000 ns
    // (hw → host offset = 5000, no drift)
    clk.observe(1000000, 1005000);
    ASSERT_FALSE(clk.is_calibrated());

    clk.observe(2000000, 2005000);
    ASSERT_FALSE(clk.is_calibrated());

    clk.observe(3000000, 3005000);
    ASSERT_TRUE(clk.is_calibrated());

    // Converted value should be close to hw + 5000
    int64_t result = clk.hw_to_host(4000000);
    ASSERT_NEAR(result, 4005000, 100);
    PASS();
}

void test_clock_unified_timestamp() {
    TEST_CASE("ClockService — unified_timestamp uses model when calibrated");
    ClockServiceConfig cfg;
    cfg.ols_window = 10;
    ClockService clk(cfg);

    // Not calibrated: should return host_ns
    ASSERT_EQ(clk.unified_timestamp(1000, 2000), 2000);

    // Calibrate with offset = 10000
    clk.observe(100000, 110000);
    clk.observe(200000, 210000);
    clk.observe(300000, 310000);

    ASSERT_TRUE(clk.is_calibrated());

    // Now unified_timestamp should use hw_to_host
    int64_t result = clk.unified_timestamp(400000, 410000);
    ASSERT_NEAR(result, 410000, 100);
    PASS();
}

void test_clock_drift_detection() {
    TEST_CASE("ClockService — detects drift");
    ClockServiceConfig cfg;
    cfg.ols_window = 10;
    ClockService clk(cfg);

    // Feed observations with a small but consistent drift
    // (α ≈ 1.001, meaning 1 ppm = 1 μs/s drift)
    for (int i = 0; i < 10; ++i) {
        int64_t hw = static_cast<int64_t>(i) * 1'000'000'000LL;  // 1s intervals
        int64_t host = hw + 5000 + static_cast<int64_t>(i) * 1000;  // 1μs/s drift
        clk.observe(hw, host);
    }

    double drift = clk.drift_ns_per_sec();
    // drift should be approximately 1000 ns/s (= 1 μs/s)
    ASSERT_NEAR(drift, 1000.0, 200.0);
    PASS();
}

void test_clock_jump_detection() {
    TEST_CASE("ClockService — detects time jump");
    ClockServiceConfig cfg;
    cfg.ols_window = 20;
    cfg.jump_threshold_ns = 1'000'000;  // 1 ms
    cfg.jump_confirm_count = 3;
    ClockService clk(cfg);

    bool jump_detected = false;
    clk.on_event([&](ClockEvent e, const ClockDiagnostics&) {
        if (e == ClockEvent::TimeJump) jump_detected = true;
    });

    // Calibrate normally
    for (int i = 0; i < 10; ++i) {
        clk.observe(i * 1000000LL, i * 1000000LL + 5000);
    }

    // Now introduce a time jump (offset shifts by 10 ms)
    for (int i = 10; i < 20; ++i) {
        clk.observe(i * 1000000LL, i * 1000000LL + 10'005'000);
    }

    ASSERT_TRUE(jump_detected);
    auto diag = clk.diagnostics();
    ASSERT_TRUE(diag.jumps_detected > 0);
    PASS();
}

void test_clock_reset() {
    TEST_CASE("ClockService — reset clears state");
    ClockService clk;
    clk.observe(1000000, 1005000);
    clk.observe(2000000, 2005000);
    clk.observe(3000000, 3005000);
    ASSERT_TRUE(clk.is_calibrated());

    clk.reset();
    ASSERT_FALSE(clk.is_calibrated());
    ASSERT_EQ(clk.diagnostics().observations, 0u);
    PASS();
}

void test_clock_zero_hw_ignored() {
    TEST_CASE("ClockService — hw_ns=0 is ignored");
    ClockService clk;
    clk.observe(0, 1000000);  // should be silently ignored
    clk.observe(0, 2000000);
    clk.observe(0, 3000000);
    ASSERT_FALSE(clk.is_calibrated());
    PASS();
}

void test_clock_diagnostics() {
    TEST_CASE("ClockService — diagnostics populated");
    ClockService clk;
    clk.observe(1000000, 1005000);
    clk.observe(2000000, 2005000);
    clk.observe(3000000, 3005000);

    auto diag = clk.diagnostics();
    ASSERT_EQ(diag.observations, 3u);
    ASSERT_TRUE(diag.calibrated);
    ASSERT_NEAR(diag.offset_ns, 5000.0, 100.0);
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════════
//  LidarFrameAssembler tests
// ═════════════════════════════════════════════════════════════════════════════

/// Helper: create a per-packet LidarFrame with N points.
static std::shared_ptr<LidarFrame> make_lidar_frame(
    int64_t hw_ns, uint32_t npts = 100)
{
    auto f = std::make_shared<LidarFrame>();
    f->timestamp = Timestamp{hw_ns};
    f->host_timestamp = Timestamp{hw_ns + 1000};
    f->sequence_number = 0;
    for (uint32_t i = 0; i < npts; ++i) {
        LidarPoint pt;
        pt.x = static_cast<float>(i) * 0.01f;
        pt.y = static_cast<float>(i) * 0.02f;
        pt.z = 0.0f;
        pt.intensity = 128.0f;
        pt.ring = static_cast<uint8_t>(i % 16);
        f->points.push_back(pt);
    }
    return f;
}

void test_assembler_azimuth_wrap() {
    TEST_CASE("FrameAssembler — azimuth wrap triggers frame emission");
    FrameAssemblerConfig cfg;
    cfg.min_points_to_emit = 10;
    cfg.expected_rate_hz = 10.0;
    LidarFrameAssembler asm_(cfg);

    int frames_emitted = 0;
    size_t total_points = 0;
    asm_.on_frame([&](auto pcf, const FrameAssemblyMeta&) {
        ++frames_emitted;
        total_points = pcf->points.size();
    });

    // Simulate one revolution: 0° → 360° in 8 packets
    for (int i = 0; i < 8; ++i) {
        float az_start = static_cast<float>(i) * 45.0f;
        float az_end   = az_start + 45.0f;
        auto frame = make_lidar_frame(i * 10'000'000LL, 50);
        asm_.feed(frame, az_start, az_end);
    }

    // No frame yet (haven't wrapped)
    ASSERT_EQ(frames_emitted, 0);

    // Now feed a packet that wraps past 360° → back to 0°
    auto wrap_frame = make_lidar_frame(80'000'000LL, 50);
    asm_.feed(wrap_frame, 5.0f, 50.0f);

    ASSERT_EQ(frames_emitted, 1);
    ASSERT_EQ(total_points, 400u);  // 8 packets × 50 points
    PASS();
}

void test_assembler_runt_discard() {
    TEST_CASE("FrameAssembler — runt frames discarded");
    FrameAssemblerConfig cfg;
    cfg.min_points_to_emit = 500;  // high threshold
    LidarFrameAssembler asm_(cfg);

    int frames_emitted = 0;
    asm_.on_frame([&](auto, const FrameAssemblyMeta&) {
        ++frames_emitted;
    });

    // Feed a small number of points then trigger wrap
    auto f1 = make_lidar_frame(0, 10);
    asm_.feed(f1, 350.0f, 355.0f);

    auto f2 = make_lidar_frame(1000000, 10);
    asm_.feed(f2, 5.0f, 10.0f);  // wrap → emit

    // Frame should be discarded because 10 < 500
    ASSERT_EQ(frames_emitted, 0);
    ASSERT_EQ(asm_.stats().runt_frames_discarded, 1u);
    PASS();
}

void test_assembler_dt_ns_populated() {
    TEST_CASE("FrameAssembler — dt_ns populated in output points");
    FrameAssemblerConfig cfg;
    cfg.min_points_to_emit = 10;
    LidarFrameAssembler asm_(cfg);

    std::shared_ptr<const odom::PointCloudFrame> captured;
    asm_.on_frame([&](auto pcf, const FrameAssemblyMeta&) {
        captured = pcf;
    });

    // Packet 1: hw_ns = 0, azimuth 0-180
    auto f1 = make_lidar_frame(0, 50);
    asm_.feed(f1, 0.0f, 180.0f);

    // Packet 2: hw_ns = 50ms, azimuth 180-355
    auto f2 = make_lidar_frame(50'000'000, 50);
    asm_.feed(f2, 180.0f, 355.0f);

    // Wrap → triggers emission
    auto f3 = make_lidar_frame(100'000'000, 50);
    asm_.feed(f3, 2.0f, 45.0f);

    ASSERT_TRUE(captured != nullptr);
    // First packet's points should have dt_ns = 0
    ASSERT_EQ(captured->points[0].dt_ns, 0);
    // Second packet's points should have dt_ns ≈ 50ms
    ASSERT_NEAR(captured->points[50].dt_ns, 50'000'000, 1'000'000);
    PASS();
}

void test_assembler_partial_timeout() {
    TEST_CASE("FrameAssembler — partial timeout emits accumulated data");
    FrameAssemblerConfig cfg;
    cfg.min_points_to_emit = 10;
    cfg.expected_rate_hz = 10.0;  // 100ms period
    cfg.partial_timeout_mul = 2.0;  // timeout at 200ms
    LidarFrameAssembler asm_(cfg);

    int frames_emitted = 0;
    bool was_partial = false;
    asm_.on_frame([&](auto, const FrameAssemblyMeta& meta) {
        ++frames_emitted;
        was_partial = meta.is_partial;
    });

    // Feed some data
    auto f1 = make_lidar_frame(0, 100);
    asm_.feed(f1, 0.0f, 45.0f);

    // Simulate timeout check  300ms later (> 200ms timeout)
    asm_.check_timeout(300'000'000);

    ASSERT_EQ(frames_emitted, 1);
    ASSERT_TRUE(was_partial);
    PASS();
}

void test_assembler_stats() {
    TEST_CASE("FrameAssembler — statistics tracking");
    FrameAssemblerConfig cfg;
    cfg.min_points_to_emit = 10;
    LidarFrameAssembler asm_(cfg);
    asm_.on_frame([](auto, const FrameAssemblyMeta&) {});

    auto f1 = make_lidar_frame(0, 100);
    asm_.feed(f1, 0.0f, 180.0f);

    auto f2 = make_lidar_frame(50'000'000, 100);
    asm_.feed(f2, 180.0f, 355.0f);

    // Trigger wrap
    auto f3 = make_lidar_frame(100'000'000, 50);
    asm_.feed(f3, 5.0f, 45.0f);

    auto stats = asm_.stats();
    ASSERT_EQ(stats.frames_emitted, 1u);
    ASSERT_EQ(stats.packets_ingested, 3u);
    ASSERT_EQ(stats.points_ingested, 250u);
    PASS();
}

void test_assembler_reset() {
    TEST_CASE("FrameAssembler — reset clears accumulator");
    FrameAssemblerConfig cfg;
    cfg.min_points_to_emit = 10;
    LidarFrameAssembler asm_(cfg);

    auto f1 = make_lidar_frame(0, 100);
    asm_.feed(f1, 0.0f, 180.0f);

    asm_.reset();

    // After reset, a wrap should not emit (no accumulated data)
    int frames_emitted = 0;
    asm_.on_frame([&](auto, const FrameAssemblyMeta&) {
        ++frames_emitted;
    });

    auto f2 = make_lidar_frame(100'000'000, 50);
    asm_.feed(f2, 5.0f, 45.0f);

    ASSERT_EQ(frames_emitted, 0);  // no wrap detected (fresh state)
    PASS();
}

void test_assembler_timed_mode() {
    TEST_CASE("FrameAssembler — time-based completion mode");
    FrameAssemblerConfig cfg;
    cfg.mode = CompletionMode::TimeBased;
    cfg.frame_period_ns = 100'000'000;  // 100ms
    cfg.min_points_to_emit = 10;
    LidarFrameAssembler asm_(cfg);

    int frames_emitted = 0;
    asm_.on_frame([&](auto, const FrameAssemblyMeta&) {
        ++frames_emitted;
    });

    // Feed several packets within 100ms window
    for (int i = 0; i < 5; ++i) {
        auto f = make_lidar_frame(i * 20'000'000LL, 50);
        asm_.feed_timed(f);
    }

    ASSERT_EQ(frames_emitted, 0);  // Not yet past 100ms

    // Feed one past the window
    auto f_late = make_lidar_frame(120'000'000LL, 50);
    asm_.feed_timed(f_late);

    ASSERT_EQ(frames_emitted, 1);
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════════
//  ClockService ↔ SyncEngine integration tests
// ═════════════════════════════════════════════════════════════════════════════

void test_sync_engine_uses_clock_service() {
    TEST_CASE("SyncEngine — uses ClockService for matching");

    // ── Calibrate ClockService with identity mapping (a=1, b=0) ─────
    ClockServiceConfig ccfg;
    ccfg.ols_window = 10;
    ClockService clk(ccfg);
    for (int i = 0; i < 5; ++i) {
        clk.observe((i + 1) * 1'000'000LL, (i + 1) * 1'000'000LL);
    }
    ASSERT_TRUE(clk.is_calibrated());

    // ── Build a SyncEngine with tight tolerance ─────────────────────
    SyncConfig sc;
    sc.tolerance_ns = 20'000'000;   // 20 ms
    sc.poll_interval_ms = 2;

    // ── Test 1: WITHOUT ClockService, host timestamps are far apart
    //            (200 ms) → should NOT produce a bundle. ─────────────
    {
        SyncEngine engine(sc);
        // No set_clock_service() — resolve() falls back to host_timestamp.

        std::atomic<int> count{0};
        engine.set_callback([&](std::shared_ptr<const SyncBundle>) { ++count; });
        engine.start();

        // HW timestamps are close (5 ms apart), but host timestamps
        // are 200 ms apart — well beyond the 20 ms tolerance.
        auto lidar = std::make_shared<LidarFrame>();
        lidar->timestamp      = Timestamp{100'000'000};
        lidar->host_timestamp = Timestamp{1'000'000'000};  // 1.0 s
        lidar->points.resize(10);

        auto imu = std::make_shared<ImuSample>();
        imu->timestamp      = Timestamp{101'000'000};      // hw +1 ms
        imu->host_timestamp = Timestamp{1'200'000'000};     // host +200 ms

        auto cam = std::make_shared<CameraFrame>();
        cam->timestamp      = Timestamp{105'000'000};       // hw +5 ms
        cam->host_timestamp = Timestamp{1'200'000'000};     // host +200 ms
        cam->width = 2; cam->height = 2;

        engine.feed_lidar(lidar);
        engine.feed_imu(imu);
        engine.feed_camera(cam);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        engine.stop();

        ASSERT_TRUE(count == 0);  // must NOT match — host gap > tolerance
    }

    // ── Test 2: WITH ClockService, unified_timestamp() maps the HW
    //            timestamps (which are 5 ms apart) → SHOULD match. ───
    {
        SyncEngine engine(sc);
        engine.set_clock_service(&clk);

        std::atomic<int> count{0};
        engine.set_callback([&](std::shared_ptr<const SyncBundle>) { ++count; });
        engine.start();

        // Same frames as above: hw close, host far apart.
        auto lidar = std::make_shared<LidarFrame>();
        lidar->timestamp      = Timestamp{100'000'000};
        lidar->host_timestamp = Timestamp{1'000'000'000};
        lidar->points.resize(10);

        auto imu = std::make_shared<ImuSample>();
        imu->timestamp      = Timestamp{101'000'000};
        imu->host_timestamp = Timestamp{1'200'000'000};

        auto cam = std::make_shared<CameraFrame>();
        cam->timestamp      = Timestamp{105'000'000};
        cam->host_timestamp = Timestamp{1'200'000'000};
        cam->width = 2; cam->height = 2;

        engine.feed_lidar(lidar);
        engine.feed_imu(imu);
        engine.feed_camera(cam);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        engine.stop();

        ASSERT_TRUE(count >= 1);  // MUST match — ClockService uses hw timestamps
    }

    PASS();
}

void test_time_sync_engine_uses_clock_service() {
    TEST_CASE("TimeSyncEngine — reads drift from ClockService");

    // Create a ClockService and calibrate with known drift
    ClockServiceConfig ccfg;
    ccfg.ols_window = 20;
    ClockService clk(ccfg);

    // Feed observations with 10 ppm drift → drift_ns_per_sec ≈ 10000
    for (int i = 0; i < 10; ++i) {
        int64_t hw = static_cast<int64_t>(i) * 1'000'000'000LL;
        int64_t host = hw + static_cast<int64_t>(i) * 10'000;
        clk.observe(hw, host);
    }
    ASSERT_TRUE(clk.is_calibrated());

    // Wire into TimeSyncEngine
    data::TimeSyncConfig tcfg;
    tcfg.camera_tolerance_ns = 1'000'000'000;
    tcfg.poll_interval_ms = 1;
    tcfg.drift_window = 10;
    tcfg.drift_warn_ns_per_sec = 500'000;

    data::TimeSyncEngine engine(tcfg);
    engine.set_clock_service(&clk);

    // Feed some data to produce a frame
    data::LidarFrame lf;
    lf.timestamp_ns = 0;
    lf.sequence = 0;
    lf.points.push_back({0, 0, 0, 1.0f, 0});

    data::ImageFrame cf;
    cf.timestamp_ns = 1'000'000;
    cf.width = 2; cf.height = 2; cf.stride = 6;
    cf.format = data::PixelFormat::RGB8;
    cf.data = std::make_shared<const std::vector<uint8_t>>(12, 128);

    engine.feedLidar(lf);
    engine.feedCamera(cf);

    engine.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    engine.stop();

    auto st = engine.stats();
    ASSERT_TRUE(st.frames_produced >= 1);
    // Drift should come from ClockService, not from internal OLS
    // ClockService drift is ~10000 ns/s for the observations above
    ASSERT_TRUE(std::abs(st.drift_ns_per_sec) > 1000);
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════════

int main() {
    std::puts("=== Phase 2: Hardware Integration Tests ===\n");

    std::puts("--- SequenceTracker ---");
    test_sequence_tracker_sequential();
    test_sequence_tracker_gap();
    test_sequence_tracker_wrap();
    test_sequence_tracker_large_gap_reset();
    test_sequence_tracker_reset();

    std::puts("\n--- VLP-16 Decoder ---");
    test_vlp16_valid_packet();
    test_vlp16_wrong_size();
    test_vlp16_bad_flag();
    test_vlp16_xyz_range();
    test_vlp16_azimuth_tracking();
    test_vlp16_reset();

    std::puts("\n--- DecoderFactory ---");
    test_factory_native();
    test_factory_vlp16();
    test_factory_case_insensitive();
    test_factory_unknown();
    test_factory_empty();

    std::puts("\n--- ClockService ---");
    test_clock_uncalibrated();
    test_clock_calibration();
    test_clock_unified_timestamp();
    test_clock_drift_detection();
    test_clock_jump_detection();
    test_clock_reset();
    test_clock_zero_hw_ignored();
    test_clock_diagnostics();

    std::puts("\n--- LidarFrameAssembler ---");
    test_assembler_azimuth_wrap();
    test_assembler_runt_discard();
    test_assembler_dt_ns_populated();
    test_assembler_partial_timeout();
    test_assembler_stats();
    test_assembler_reset();
    test_assembler_timed_mode();

    std::puts("\n--- ClockService ↔ SyncEngine Integration ---");
    test_sync_engine_uses_clock_service();
    test_time_sync_engine_uses_clock_service();

    std::printf("\n%d / %d tests passed\n", g_tests_passed, g_tests_run);
    if (g_tests_passed == g_tests_run) {
        std::puts("\nPhase 2 Hardware Integration: ALL TESTS PASSED");
        return 0;
    }
    return 1;
}
