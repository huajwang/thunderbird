// ─────────────────────────────────────────────────────────────────────────────
// Test — Wire protocol, CRC, and PacketParser
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/protocol.h"
#include "thunderbird/packet_parser.h"

#include <cassert>
#include <cstdio>
#include <cstring>
#include <memory>

using namespace thunderbird;
using namespace thunderbird::protocol;

// ── CRC-32 basic tests ──────────────────────────────────────────────────────

void test_crc32() {
    // CRC of empty data
    assert(Crc32::compute(nullptr, 0) == 0x00000000);

    // CRC of known data
    const uint8_t data[] = "123456789";
    (void)data;
    assert(Crc32::compute(data, 9) == 0xCBF43926);  // standard CRC-32 test vector

    std::puts("  CRC-32: OK");
}

// ── Packet header layout ────────────────────────────────────────────────────

void test_header_size() {
    assert(sizeof(PacketHeader) == kHeaderSize);
    assert(kHeaderSize == 20);
    std::puts("  Header size: OK");
}

// ── build_packet + validation ───────────────────────────────────────────────

void test_build_and_validate() {
    uint8_t buf[256];

    // Build a heartbeat packet (no extra payload)
    HeartbeatPayload hb{};
    hb.sender_timestamp_ns = 12345;
    hb.uptime_seconds      = 42;
    hb.error_flags         = 0;

    size_t total = build_packet(buf, sizeof(buf),
                                PacketType::Heartbeat, 1, 99999,
                                reinterpret_cast<const uint8_t*>(&hb), sizeof(hb));
    (void)total;

    assert(total == kHeaderSize + sizeof(hb) + kCrcSize);
    assert(total <= sizeof(buf));

    // Validate CRC
    assert(Crc32::validate(buf, total));

    // Corrupt one byte → CRC should fail
    buf[5] ^= 0xFF;
    assert(!Crc32::validate(buf, total));

    std::puts("  build_packet + CRC validate: OK");
}

// ── PacketParser: IMU round-trip ────────────────────────────────────────────

void test_parser_imu() {
    PacketParser parser;
    int callback_count = 0;
    std::shared_ptr<const ImuSample> last;

    parser.on_imu([&](std::shared_ptr<const ImuSample> s) {
        ++callback_count;
        last = s;
    });

    ImuWirePayload wire;
    wire.accel_x = 1.0f; wire.accel_y = 2.0f; wire.accel_z = 9.81f;
    wire.gyro_x  = 0.1f; wire.gyro_y  = 0.2f; wire.gyro_z  = 0.3f;
    wire.temperature = 25.0f;

    uint8_t pkt[128];
    size_t n = build_packet(pkt, sizeof(pkt),
                            PacketType::ImuSample, 7, 1'000'000'000,
                            reinterpret_cast<const uint8_t*>(&wire), sizeof(wire));

    parser.feed(pkt, n);

    assert(callback_count == 1);
    assert(last != nullptr);
    assert(last->accel[0] == 1.0f);
    assert(last->accel[1] == 2.0f);
    assert(last->accel[2] == 9.81f);
    assert(last->gyro[0]  == 0.1f);
    assert(last->temperature == 25.0f);
    assert(last->timestamp.nanoseconds == 1'000'000'000);

    std::puts("  Parser IMU round-trip: OK");
}

// ── PacketParser: LiDAR round-trip ──────────────────────────────────────────

void test_parser_lidar() {
    PacketParser parser;
    int callback_count = 0;
    std::shared_ptr<const LidarFrame> last;

    parser.on_lidar([&](std::shared_ptr<const LidarFrame> f) {
        ++callback_count;
        last = f;
    });

    constexpr int kN = 10;
    LidarSubHeader sub;
    sub.num_points    = kN;
    sub.azimuth_start = 0.0f;
    sub.azimuth_end   = 180.0f;

    LidarWirePoint pts[kN];
    for (int i = 0; i < kN; ++i) {
        pts[i].x = static_cast<float>(i);
        pts[i].y = static_cast<float>(i) * 2;
        pts[i].z = 1.5f;
        pts[i].intensity = static_cast<uint8_t>(i * 10);
        pts[i].ring = static_cast<uint8_t>(i % 4);
    }

    // Assemble payload: sub-header + points
    uint8_t payload[sizeof(sub) + sizeof(pts)];
    std::memcpy(payload, &sub, sizeof(sub));
    std::memcpy(payload + sizeof(sub), pts, sizeof(pts));

    uint8_t pkt[2048];
    size_t n = build_packet(pkt, sizeof(pkt),
                            PacketType::LidarScan, 42, 2'000'000'000,
                            payload, sizeof(payload));

    parser.feed(pkt, n);

    assert(callback_count == 1);
    assert(last != nullptr);
    assert(last->points.size() == kN);
    assert(last->sequence_number == 42);
    assert(last->timestamp.nanoseconds == 2'000'000'000);
    assert(last->points[3].x == 3.0f);
    assert(last->points[3].ring == 3);

    std::puts("  Parser LiDAR round-trip: OK");
}

// ── PacketParser: partial delivery (split bytes) ────────────────────────────

void test_parser_partial_feed() {
    PacketParser parser;
    int callback_count = 0;

    parser.on_imu([&](auto) { ++callback_count; });

    ImuWirePayload wire{};
    wire.accel_z = 9.81f;

    uint8_t pkt[128];
    size_t n = build_packet(pkt, sizeof(pkt),
                            PacketType::ImuSample, 0, 0,
                            reinterpret_cast<const uint8_t*>(&wire), sizeof(wire));

    // Feed one byte at a time — parser must accumulate correctly.
    for (size_t i = 0; i < n; ++i) {
        parser.feed(&pkt[i], 1);
    }

    assert(callback_count == 1);
    std::puts("  Parser partial feed: OK");
}

// ── PacketParser: CRC error recovery ────────────────────────────────────────

void test_parser_crc_recovery() {
    PacketParser parser;
    int callback_count = 0;

    parser.on_imu([&](auto) { ++callback_count; });

    ImuWirePayload wire{};
    wire.accel_z = 9.81f;

    // Build a valid packet
    uint8_t good_pkt[128];
    size_t good_n = build_packet(good_pkt, sizeof(good_pkt),
                                 PacketType::ImuSample, 0, 0,
                                 reinterpret_cast<const uint8_t*>(&wire), sizeof(wire));

    // Build a corrupted packet
    uint8_t bad_pkt[128];
    size_t bad_n = build_packet(bad_pkt, sizeof(bad_pkt),
                                PacketType::ImuSample, 0, 0,
                                reinterpret_cast<const uint8_t*>(&wire), sizeof(wire));
    bad_pkt[10] ^= 0xFF;  // corrupt a byte

    // Feed: bad, then good.
    parser.feed(bad_pkt, bad_n);
    parser.feed(good_pkt, good_n);

    // Should have recovered and parsed the good packet.
    assert(callback_count == 1);
    assert(parser.stats().crc_errors >= 1);
    std::puts("  Parser CRC recovery: OK");
}

// ── PacketParser: multiple packets in one feed ──────────────────────────────

void test_parser_multiple_packets() {
    PacketParser parser;
    int imu_n = 0;

    parser.on_imu([&](auto) { ++imu_n; });

    ImuWirePayload wire{};
    wire.accel_z = 9.81f;

    // Concatenate 5 packets into one big buffer
    uint8_t big_buf[5 * 128];
    size_t offset = 0;
    for (int i = 0; i < 5; ++i) {
        size_t n = build_packet(big_buf + offset, sizeof(big_buf) - offset,
                                PacketType::ImuSample, static_cast<uint32_t>(i), 0,
                                reinterpret_cast<const uint8_t*>(&wire), sizeof(wire));
        offset += n;
    }

    // Feed all at once
    parser.feed(big_buf, offset);
    assert(imu_n == 5);

    std::puts("  Parser multiple packets: OK");
}

// ─────────────────────────────────────────────────────────────────────────────

int main() {
    std::puts("=== Protocol & Parser Tests ===");

    test_crc32();
    test_header_size();
    test_build_and_validate();
    test_parser_imu();
    test_parser_lidar();
    test_parser_partial_feed();
    test_parser_crc_recovery();
    test_parser_multiple_packets();

    std::puts("\nProtocol & Parser: ALL TESTS PASSED");
    return 0;
}
