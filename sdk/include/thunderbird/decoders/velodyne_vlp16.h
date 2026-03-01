// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Velodyne VLP-16 packet decoder
// ─────────────────────────────────────────────────────────────────────────────
//
// Decodes Velodyne VLP-16 UDP data packets (1206 bytes each) into SDK
// LidarFrame objects.  Zero-copy on the hot path — all parsing uses a
// fixed-size scratch buffer; only the SDK output LidarFrame heap-allocates.
//
// Wire format (VLP-16 Data Packet, 1206 bytes):
//   12 firing blocks × 100 bytes each:
//     [0..1]  flag       (0xFFEE)
//     [2..3]  azimuth    (0.01° resolution)
//     [4..99] 32 channels × 3 bytes (distance_u16, reflectivity_u8)
//   [1200..1203]  GPS timestamp (μs since top of hour)
//   [1204]        return mode
//   [1205]        product ID
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/packet_decoder.h"
#include "thunderbird/sequence_tracker.h"

#include <array>
#include <cmath>
#include <cstring>

namespace thunderbird {

namespace vlp16 {

// ── Wire protocol constants ────────────────────────────────────────────────

inline constexpr size_t   kPacketSize        = 1206;
inline constexpr size_t   kNumFiringBlocks   = 12;
inline constexpr size_t   kChannelsPerBlock  = 32;  // 16 channels × 2 returns
inline constexpr size_t   kSingleReturnChans = 16;
inline constexpr uint16_t kFiringBlockFlag   = 0xFFEE;
inline constexpr float    kDistanceUnit      = 0.002f;  // 2 mm per count
inline constexpr float    kAzimuthUnit       = 0.01f;   // 0.01° per count
inline constexpr float    kDeg2Rad           = 3.14159265358979323846f / 180.0f;

// Fixed vertical angles for VLP-16's 16 channels (degrees)
inline constexpr std::array<float, 16> kElevationDeg = {{
    -15.0f, 1.0f, -13.0f, 3.0f, -11.0f, 5.0f, -9.0f, 7.0f,
     -7.0f, 9.0f,  -5.0f, 11.0f, -3.0f, 13.0f, -1.0f, 15.0f
}};

// ── On-wire structures (packed, no padding) ─────────────────────────────

#pragma pack(push, 1)

struct ChannelData {
    uint16_t distance;      // little-endian, 2 mm resolution
    uint8_t  reflectivity;
};
static_assert(sizeof(ChannelData) == 3);

struct FiringBlock {
    uint16_t    flag;       // 0xFFEE
    uint16_t    azimuth;    // 0.01° resolution
    ChannelData channels[kChannelsPerBlock];
};
static_assert(sizeof(FiringBlock) == 100);

struct DataPacket {
    FiringBlock blocks[kNumFiringBlocks];
    uint32_t    gps_timestamp_us;   // μs since top of hour
    uint8_t     return_mode;        // 0x37=strongest, 0x38=last, 0x39=dual
    uint8_t     product_id;         // 0x22 = VLP-16
};
static_assert(sizeof(DataPacket) == kPacketSize);

#pragma pack(pop)

} // namespace vlp16

// ─────────────────────────────────────────────────────────────────────────────

class VelodyneVlp16Decoder final : public IPacketDecoder {
public:
    VelodyneVlp16Decoder() = default;

    // ── IPacketDecoder ──────────────────────────────────────────────────

    void feed(const uint8_t* data, size_t len) override {
        stats_.bytes_processed += len;

        // Each UDP datagram is exactly 1206 bytes.
        if (len != vlp16::kPacketSize) {
            ++stats_.malformed_count;
            return;
        }

        // Validate first firing block flag as sanity check.
        uint16_t flag;
        std::memcpy(&flag, data, sizeof(flag));
        if (flag != vlp16::kFiringBlockFlag) {
            ++stats_.malformed_count;
            return;
        }

        parse_data_packet(data);
    }

    void reset() override {
        stats_ = {};
        seq_tracker_.reset();
        frame_seq_ = 0;
    }

    DecoderStats stats() const override { return stats_; }

    const char* decoder_name() const override { return "Velodyne VLP-16"; }

    /// Access last packet's azimuth range (for frame assembler integration).
    float last_azimuth_start() const { return last_azimuth_start_; }
    float last_azimuth_end()   const { return last_azimuth_end_; }

private:
    void parse_data_packet(const uint8_t* pkt) {
        // Extract GPS timestamp (offset 1200, 4 bytes LE).
        uint32_t gps_us = 0;
        std::memcpy(&gps_us, pkt + 1200, sizeof(gps_us));

        // Build timestamps.
        Timestamp hw_ts{static_cast<int64_t>(gps_us) * 1000};
        Timestamp host_ts = host_timestamp();

        float first_azimuth = 0.0f;
        float last_azimuth  = 0.0f;
        size_t point_count  = 0;

        for (size_t b = 0; b < vlp16::kNumFiringBlocks; ++b) {
            const size_t block_offset = b * sizeof(vlp16::FiringBlock);

            // Validate block flag.
            uint16_t bflag;
            std::memcpy(&bflag, pkt + block_offset, sizeof(bflag));
            if (bflag != vlp16::kFiringBlockFlag) {
                ++stats_.malformed_count;
                continue;
            }

            // Read azimuth.
            uint16_t raw_azimuth;
            std::memcpy(&raw_azimuth, pkt + block_offset + 2, sizeof(raw_azimuth));
            float azimuth_deg = static_cast<float>(raw_azimuth) * vlp16::kAzimuthUnit;
            float azimuth_rad = azimuth_deg * vlp16::kDeg2Rad;

            if (b == 0) first_azimuth = azimuth_deg;
            last_azimuth = azimuth_deg;

            // Parse 16 channels from the first firing.
            const size_t chan_offset = block_offset + 4;

            for (size_t ch = 0; ch < vlp16::kSingleReturnChans; ++ch) {
                const size_t ch_off = chan_offset + ch * sizeof(vlp16::ChannelData);

                if (ch_off + sizeof(vlp16::ChannelData) > vlp16::kPacketSize) break;

                uint16_t raw_dist;
                uint8_t  reflectivity;
                std::memcpy(&raw_dist, pkt + ch_off, sizeof(raw_dist));
                reflectivity = pkt[ch_off + 2];

                if (raw_dist == 0) continue;  // no return

                float dist_m = static_cast<float>(raw_dist) * vlp16::kDistanceUnit;
                float elev_rad = vlp16::kElevationDeg[ch] * vlp16::kDeg2Rad;

                float cos_elev = std::cos(elev_rad);
                float x = dist_m * cos_elev * std::sin(azimuth_rad);
                float y = dist_m * cos_elev * std::cos(azimuth_rad);
                float z = dist_m * std::sin(elev_rad);

                if (point_count < kMaxPointsPerPacket) {
                    auto& pt     = scratch_points_[point_count];
                    pt.x         = x;
                    pt.y         = y;
                    pt.z         = z;
                    pt.intensity = static_cast<float>(reflectivity);
                    pt.ring      = static_cast<uint8_t>(ch);
                    ++point_count;
                }
            }
        }

        if (point_count == 0) return;

        // Build SDK LidarFrame (single heap allocation).
        auto frame = std::make_shared<LidarFrame>();
        frame->timestamp       = hw_ts;
        frame->host_timestamp  = host_ts;
        frame->sequence_number = frame_seq_++;
        frame->points.assign(scratch_points_.begin(),
                             scratch_points_.begin() +
                                 static_cast<ptrdiff_t>(point_count));

        last_azimuth_start_ = first_azimuth;
        last_azimuth_end_   = last_azimuth;

        ++stats_.packets_parsed;

        if (lidar_cb_) {
            lidar_cb_(std::move(frame));
        }
    }

    // ── State ───────────────────────────────────────────────────────────

    static constexpr size_t kMaxPointsPerPacket = 192;
    std::array<LidarPoint, kMaxPointsPerPacket> scratch_points_{};

    DecoderStats     stats_{};
    SequenceTracker  seq_tracker_;
    uint32_t         frame_seq_{0};
    float            last_azimuth_start_{0.0f};
    float            last_azimuth_end_{0.0f};
};

} // namespace thunderbird
