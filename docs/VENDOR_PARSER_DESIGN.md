# Thunderbird SDK — Vendor-Specific Packet Parser Design

> **Status:** Design proposal  
> **Author:** Systems Architecture  
> **Date:** 2026-03-01  
> **Depends on:** `PacketParser`, `protocol.h`, `types.h`, `ConnectionManager`

---

## 0. Existing Code Audit

### What already exists

The codebase has a fully functional `PacketParser` (packet_parser.h) that:

- **Parses LiDAR** (x, y, z, intensity, ring) + azimuth (sub-header) — ✅
- **Parses IMU** (accel, gyro, temperature) — ✅
- **Handles partial packets** via accumulation buffer + incremental `feed()` — ✅
- **Validates checksum** via CRC-32 on every packet — ✅
- **Handles malformed packets** via magic-scan resync, version check, length cap — ✅
- **Tracks statistics** (`packets_parsed`, `crc_errors`, `resync_count`, `bytes_processed`) — ✅

### What's missing

1. **No `IPacketDecoder` interface** — `ConnectionManager` hard-codes
   `PacketParser parser_` (member, not pointer).  Can't swap vendor decoders.

2. **No packet-loss detection** — `PacketHeader::sequence` is a per-channel
   monotonic counter, but nobody tracks the previous sequence number or
   reports gaps.  Lost packets are invisible.

3. **Per-packet heap allocation** — Every `dispatch_lidar()` does
   `make_shared<LidarFrame>()` + `vector::resize()`.  For the native
   protocol on a stream transport this is acceptable (packets arrive merged
   in 64 KB reads).  For vendor UDP (one datagram = one packet at 10–20 kHz),
   this is ~20k allocations/s on the hot path.

4. **No vendor-specific decoders** — Only the Thunderbird-native wire format
   (0xBEEF magic, 20-byte header) is supported.

This design addresses all four gaps.

---

## 1. `IPacketDecoder` Interface

### 1.1 Rationale

The existing `PacketParser` is a good implementation but a concrete class.
We need a polymorphic interface so `ConnectionManager` can hold any decoder.

**Design constraints from existing code:**
- Callbacks are `LidarCallback`, `ImuCallback`, `CameraCallback` (from types.h)
- `ConnectionManager::io_loop()` calls `parser_.feed(buf, n)` on its I/O thread
- `HardwareDriver` registers callbacks via `conn_->parser().on_lidar(...)` etc.
- Single-threaded access from the I/O thread — no locking needed inside feed()

### 1.2 Interface

```cpp
// sdk/include/thunderbird/packet_decoder.h
#pragma once

#include "thunderbird/types.h"
#include <cstdint>
#include <cstddef>
#include <string>

namespace thunderbird {

// ─── Decoder statistics ─────────────────────────────────────────────────────

struct DecoderStats {
    uint64_t packets_parsed{0};      // successfully decoded
    uint64_t packets_dropped{0};     // detected by sequence gap
    uint64_t checksum_errors{0};     // failed integrity check
    uint64_t resync_count{0};        // stream re-synchronisations
    uint64_t malformed_count{0};     // structurally invalid (wrong size, bad field)
    uint64_t bytes_processed{0};     // total bytes fed
};

// ─── Abstract decoder ───────────────────────────────────────────────────────

/// Converts raw transport bytes into typed SDK sensor objects.
///
/// Implementations:
///   • PacketParser           — Thunderbird native wire protocol (existing)
///   • VelodyneVlp16Decoder   — Velodyne VLP-16 / VLP-32C / Puck
///   • OusterOs1Decoder       — Ouster OS1-64 / OS1-128
///   • LivoxMid360Decoder     — Livox Mid-360
///
/// Contract:
///   • feed() is called from a single I/O thread — no internal locking.
///   • Callbacks fire synchronously inside feed() on the I/O thread.
///   • Implementations must tolerate arbitrary byte boundaries (partial data).
///   • Internal parsing must not heap-allocate per packet on the hot path.
///     (SDK output types like LidarFrame do allocate; that's unavoidable.)
class IPacketDecoder {
public:
    virtual ~IPacketDecoder() = default;

    // ── Callback registration ───────────────────────────────────────────
    //
    // Non-virtual.  Stored in base class so all decoders share the
    // same registration surface.  ConnectionManager / HardwareDriver
    // call these without knowing the concrete decoder type.

    void on_lidar(LidarCallback cb)    { lidar_cb_  = std::move(cb); }
    void on_imu(ImuCallback cb)        { imu_cb_    = std::move(cb); }
    void on_camera(CameraCallback cb)  { camera_cb_ = std::move(cb); }

    // ── Ingestion ───────────────────────────────────────────────────────

    /// Feed raw bytes.  Parses and dispatches complete sensor frames
    /// via the registered callbacks.  Retains partial data internally
    /// for the next call.
    virtual void feed(const uint8_t* data, size_t len) = 0;

    /// Discard internal parser state (e.g. after reconnect / link loss).
    virtual void reset() = 0;

    // ── RX timestamp injection (for SO_TIMESTAMPING, Phase 2) ───────────

    /// Set the kernel/NIC RX timestamp for the NEXT feed() call.
    /// Decoders should use host_timestamp() instead of Timestamp::now()
    /// to populate frame->host_timestamp.
    /// Pass 0 to revert to software timestamps.
    void set_rx_timestamp(int64_t rx_ns) { rx_timestamp_ns_ = rx_ns; }

    // ── Metadata ────────────────────────────────────────────────────────

    virtual const DecoderStats& stats() const = 0;

    /// Human-readable name: "Thunderbird native", "Velodyne VLP-16", etc.
    virtual const char* decoder_name() const = 0;

protected:
    /// Returns the best available host timestamp.
    /// If SO_TIMESTAMPING injected a kernel RX timestamp, use it.
    /// Otherwise falls back to steady_clock::now().
    Timestamp host_timestamp() const {
        if (rx_timestamp_ns_ != 0)
            return Timestamp{rx_timestamp_ns_};
        return Timestamp::now();
    }

    LidarCallback  lidar_cb_;
    ImuCallback    imu_cb_;
    CameraCallback camera_cb_;

private:
    int64_t rx_timestamp_ns_{0};
};

} // namespace thunderbird
```

**Key decisions:**

- **Callbacks in base class, not virtual**: avoids vtable call overhead on
  every callback dispatch.  All decoders use the same `LidarCallback` /
  `ImuCallback` / `CameraCallback` types from types.h.

- **`DecoderStats` replaces `ParserStats`**: adds `packets_dropped` and
  `malformed_count`.  Old `ParserStats` fields map directly.

- **`host_timestamp()` helper**: centralises SO_TIMESTAMPING fallback.
  Single place to change when we add NIC-level timestamps.

- **No `on_control()`**: vendor protocols don't have Thunderbird control
  messages.  `PacketParser` keeps its `on_control()` as an extension method
  (not in the interface).

### 1.3 Making `PacketParser` implement `IPacketDecoder`

Minimal change — add inheritance, override the three virtuals, keep `on_control()`
as an extension:

```cpp
class PacketParser : public IPacketDecoder {
public:
    // Existing public API unchanged.
    // on_control() remains a non-virtual extension.
    void on_control(ControlCallback cb) { control_cb_ = std::move(cb); }

    // ── IPacketDecoder ──────────────────────────────────────────────────
    void feed(const uint8_t* data, size_t len) override;  // same impl
    void reset() override;
    const DecoderStats& stats() const override { return stats_; }
    const char* decoder_name() const override { return "Thunderbird native"; }

    // dispatch() changes Timestamp::now() → host_timestamp()
    // ParserStats → DecoderStats (superset, drop-in)
    // Add sequence tracking for packet-loss detection (see §3)
};
```

---

## 2. Vendor Packet Format Analysis

Before writing a "VendorX" decoder, we need to understand what real
hardware actually sends.  Here's the landscape:

### 2.1 Self-framing vs. stream-framing

| Transport | Framing model | Accumulation buffer needed? |
|-----------|--------------|---------------------------|
| UDP (LiDAR) | **Self-framing**: each recvfrom() = exactly one packet | No — parse in-place from the recv buffer |
| TCP (control) | **Stream**: bytes merge across reads | Yes — existing buf_ model |
| USB bulk | **Stream**: transfers merge | Yes |

**Critical insight:** Real LiDAR sensors overwhelmingly use **UDP**.  Each UDP
datagram is a single, self-contained packet.  The accumulation buffer
(`buf_.insert()`) that `PacketParser` uses for stream framing is **unnecessary
and harmful** for vendor UDP:
- Unnecessary: no partial packets to accumulate.
- Harmful: `buf_.insert()` copies every byte into a `vector`, then `buf_.erase()`
  shifts the remainder — O(n) memmove per packet at 10–20 kHz.

For vendor decoders, `feed()` should parse the buffer **in-place** without copying.

### 2.2 Typical vendor packet structure (Velodyne VLP-16 as reference)

```
Offset  Size    Field
──────  ──────  ──────────────────────────
  0       42    factory header (uninteresting)

  Per firing block (12 blocks, 100 bytes each):
  0        2    flag       0xFFEE
  2        2    azimuth    0.01° resolution, big-endian
  4       96    channels   32 × {distance_u16, reflectivity_u8} = 3 bytes each

 1200      4    GPS timestamp (μs since top of hour)
 1204      1    return mode
 1205      1    product ID
                ─────────
                1206 bytes total (fixed)
```

- **No CRC** — integrity relies on UDP checksum (offloaded to NIC).
- **No magic** — fixed 1206-byte datagrams; framing is implicit.
- **Azimuth** per firing block, not per point — interpolation needed.
- **Distance** is unsigned 16-bit, 2 mm resolution.
- **Timestamp** is GPS microseconds, not nanoseconds.

### 2.3 Consequence for the interface

The `feed()` contract must support both models:
- **Stream decoders** (Thunderbird native, USB): call `feed()` with arbitrary
  chunks; decoder accumulates internally.
- **Datagram decoders** (Velodyne, Ouster, Livox): call `feed()` once per
  recvfrom(); decoder parses the entire buffer in one shot, zero-copy.

The interface handles this naturally — `feed()` doesn't specify framing
semantics.  Each implementation decides whether to buffer.

---

## 3. Packet-Loss Detection

### 3.1 Current gap

`PacketHeader::sequence` is a per-channel monotonic counter.  The packet
header carries it.  Vendor packets typically have their own sequence
mechanism (Velodyne: none for data, Ouster: column and frame counters).
But **nobody tracks gaps today**.

### 3.2 Design: `SequenceTracker`

A small utility class that any decoder can embed:

```cpp
// sdk/include/thunderbird/sequence_tracker.h
#pragma once
#include <cstdint>

namespace thunderbird {

/// Tracks a per-channel monotonic sequence number and detects gaps.
/// No heap allocation.  No locking (single-threaded I/O path).
class SequenceTracker {
public:
    /// Record a received sequence number.
    /// Returns the number of dropped packets (0 = no loss).
    uint32_t update(uint32_t seq) {
        if (!initialised_) {
            prev_seq_ = seq;
            initialised_ = true;
            return 0;
        }

        // Expected: prev + 1.  Handle wrap-around (uint32 counter).
        uint32_t expected = prev_seq_ + 1;
        uint32_t gap = seq - expected;  // unsigned subtraction handles wrap

        prev_seq_ = seq;

        // If gap > threshold, treat as re-initialisation (device restart)
        // rather than massive loss.
        if (gap > kMaxReasonableGap) {
            total_resets_++;
            return 0;
        }

        total_dropped_ += gap;
        return gap;
    }

    void reset() {
        initialised_ = false;
        prev_seq_ = 0;
        total_dropped_ = 0;
        total_resets_ = 0;
    }

    uint64_t total_dropped() const { return total_dropped_; }
    uint32_t total_resets()  const { return total_resets_; }

private:
    static constexpr uint32_t kMaxReasonableGap = 1000;

    bool     initialised_{false};
    uint32_t prev_seq_{0};
    uint64_t total_dropped_{0};
    uint32_t total_resets_{0};
};

} // namespace thunderbird
```

**Usage in PacketParser** (native protocol — per-channel tracking):

```cpp
// One tracker per PacketType channel:
SequenceTracker lidar_seq_;
SequenceTracker imu_seq_;
SequenceTracker camera_seq_;

void dispatch(const protocol::PacketHeader& hdr, ...) {
    ...
    switch (ptype) {
    case PacketType::LidarScan: {
        uint32_t dropped = lidar_seq_.update(hdr.sequence);
        stats_.packets_dropped += dropped;
        dispatch_lidar(...);
        break;
    }
    ...
    }
}
```

**Usage in vendor decoders:** Vendor-specific sequence fields
(Ouster has `measurement_id`, Hesai has `block_id`) get their own
`SequenceTracker` instance.

---

## 4. VelodyneVlp16Decoder — Concrete Implementation

### 4.1 Wire format constants (no dynamic memory)

```cpp
// sdk/include/thunderbird/decoders/velodyne_vlp16.h
#pragma once

#include "thunderbird/packet_decoder.h"
#include "thunderbird/sequence_tracker.h"

#include <array>
#include <cstring>
#include <cmath>

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
    -7.0f, 9.0f, -5.0f, 11.0f, -3.0f, 13.0f, -1.0f, 15.0f
}};

// ── On-wire structures (packed, no padding) ─────────────────────────────

#pragma pack(push, 1)

struct ChannelData {
    uint16_t distance;      // big-endian or little-endian depending on model
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

        // ── Step 1: bounds check ────────────────────────────────────────
        // Each UDP datagram is exactly 1206 bytes.  Anything else is junk.
        if (len != vlp16::kPacketSize) {
            ++stats_.malformed_count;
            return;
        }

        // ── Step 2: validate structure (no CRC on Velodyne packets) ─────
        // Check the first firing block flag as a sanity test.
        // Not a full checksum, but the flag is a constant 0xFFEE.
        uint16_t flag;
        std::memcpy(&flag, data, sizeof(flag));
        if (flag != vlp16::kFiringBlockFlag) {
            ++stats_.malformed_count;
            return;
        }

        // ── Step 3: overlay the wire struct for zero-copy access ────────
        // IMPORTANT: we do NOT memcpy the whole 1206 bytes into a local
        // DataPacket.  We read fields individually via safe memcpy to
        // avoid UB from aliasing and alignment.
        //
        // For firing blocks, we iterate by computed offsets.
        parse_data_packet(data);
    }

    void reset() override {
        stats_ = {};
        seq_tracker_.reset();
        frame_azimuth_start_ = 0.0f;
        frame_azimuth_end_   = 0.0f;
        frame_seq_           = 0;
    }

    const DecoderStats& stats() const override { return stats_; }

    const char* decoder_name() const override { return "Velodyne VLP-16"; }

private:
    // ── Packet parsing (hot path — no heap allocation) ──────────────────

    void parse_data_packet(const uint8_t* pkt) {
        // Extract GPS timestamp (offset 1200, 4 bytes LE)
        uint32_t gps_us = 0;
        std::memcpy(&gps_us, pkt + 1200, sizeof(gps_us));

        // Build hardware timestamp (GPS μs → nanoseconds)
        Timestamp hw_ts{static_cast<int64_t>(gps_us) * 1000};
        Timestamp host_ts = host_timestamp();

        // Accumulate points from all 12 firing blocks into frame_points_.
        // Each firing block has 32 channel entries.  For single-return mode,
        // channels 0–15 are the first firing and 16–31 are the second.
        // We treat them as 16 channels with interleaved firings.

        // Track azimuth range across the packet for the LidarFrame sub-header.
        float first_azimuth = 0.0f;
        float last_azimuth  = 0.0f;

        size_t point_count = 0;

        for (size_t b = 0; b < vlp16::kNumFiringBlocks; ++b) {
            const size_t block_offset = b * sizeof(vlp16::FiringBlock);

            // Read and validate the block flag.
            uint16_t bflag;
            std::memcpy(&bflag, pkt + block_offset, sizeof(bflag));
            if (bflag != vlp16::kFiringBlockFlag) {
                ++stats_.malformed_count;
                continue;
            }

            // Read azimuth (offset +2 in the block)
            uint16_t raw_azimuth;
            std::memcpy(&raw_azimuth, pkt + block_offset + 2, sizeof(raw_azimuth));
            float azimuth_deg = static_cast<float>(raw_azimuth) * vlp16::kAzimuthUnit;
            float azimuth_rad = azimuth_deg * vlp16::kDeg2Rad;

            if (b == 0) first_azimuth = azimuth_deg;
            last_azimuth = azimuth_deg;

            // Parse the 16 channels from the first firing in this block.
            // (For dual-return mode, channels 16-31 are a second return —
            //  skip them for now; add dual-return support later.)
            const size_t chan_offset = block_offset + 4; // past flag + azimuth

            for (size_t ch = 0; ch < vlp16::kSingleReturnChans; ++ch) {
                const size_t ch_off = chan_offset + ch * sizeof(vlp16::ChannelData);

                // Bounds check: ensure we don't read past the packet.
                if (ch_off + sizeof(vlp16::ChannelData) > vlp16::kPacketSize) {
                    break;
                }

                uint16_t raw_dist;
                uint8_t  reflectivity;
                std::memcpy(&raw_dist, pkt + ch_off, sizeof(raw_dist));
                reflectivity = pkt[ch_off + 2];

                // Skip zero-distance (no return)
                if (raw_dist == 0) continue;

                float dist_m = static_cast<float>(raw_dist) * vlp16::kDistanceUnit;
                float elev_rad = vlp16::kElevationDeg[ch] * vlp16::kDeg2Rad;

                // Spherical → Cartesian
                float cos_elev = std::cos(elev_rad);
                float x = dist_m * cos_elev * std::sin(azimuth_rad);
                float y = dist_m * cos_elev * std::cos(azimuth_rad);
                float z = dist_m * std::sin(elev_rad);

                // Write into pre-allocated scratch buffer (no heap alloc)
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

        // ── Build SDK LidarFrame (single heap allocation here) ──────────
        //
        // This is the only allocation in the entire parse path.
        // The scratch_points_ array is stack-resident.
        auto frame = std::make_shared<LidarFrame>();
        frame->timestamp       = hw_ts;
        frame->host_timestamp  = host_ts;
        frame->sequence_number = frame_seq_++;
        frame->points.assign(scratch_points_.begin(),
                             scratch_points_.begin() +
                                 static_cast<ptrdiff_t>(point_count));

        // Sequence tracking (using our synthetic frame counter —
        // VLP-16 has no per-packet sequence number)
        // For real gap detection, use azimuth continuity instead.

        ++stats_.packets_parsed;

        if (lidar_cb_) {
            lidar_cb_(std::move(frame));
        }
    }

    // ── State (no heap, all fixed-size) ─────────────────────────────────

    /// Scratch buffer for points: 12 blocks × 16 channels = 192 max.
    static constexpr size_t kMaxPointsPerPacket = 192;
    std::array<LidarPoint, kMaxPointsPerPacket> scratch_points_{};

    DecoderStats     stats_{};
    SequenceTracker  seq_tracker_;
    float            frame_azimuth_start_{0.0f};
    float            frame_azimuth_end_{0.0f};
    uint32_t         frame_seq_{0};
};

} // namespace thunderbird
```

### 4.2 IMU handling for vendors

The VLP-16 does not produce its own IMU data (IMU is a separate device).
Vendors like Ouster embed IMU in a separate UDP port/packet type.

For multi-port vendors, the `ConnectionManager` would hold **two** decoders
(lidar decoder + imu decoder), or the decoder implementation internally
demuxes by inspecting packet size/header.  That's an implementation detail —
the `IPacketDecoder` interface supports it either way.

---

## 5. State Machine Diagram

### 5.1 Thunderbird native protocol (stream-framed, existing `PacketParser`)

```
                                    feed(data, len)
                                         │
                                         ▼
                        ┌──────────────────────────────┐
                        │         ACCUMULATE            │
                        │   buf_.insert(data, len)      │
                        └──────────────┬───────────────┘
                                       │
                                       ▼
                   ┌───────── buf_.size() >= 20? ──────────┐
                   │ no                                    │ yes
                   ▼                                       ▼
              ┌─────────┐                    ┌──────────────────────┐
              │  WAIT    │                    │    SCAN_FOR_MAGIC    │
              │  (return │                    │    find_magic()      │
              │   false) │                    └───────────┬──────────┘
              └─────────┘                                │
                                        ┌────────────────┴────────────┐
                                   not found                      found at
                                        │                        offset >= 0
                                        ▼                             │
                              ┌───────────────┐              ┌───────┴────────┐
                              │ DISCARD_JUNK   │              │ offset > 0?    │
                              │ keep last byte │              └───┬─────┬──────┘
                              │ resync_count++ │               no │     │ yes
                              │ return false   │                  │     ▼
                              └───────────────┘                  │  DISCARD prefix
                                                                 │  resync_count++
                                                                 │     │
                                                        ┌────────┘     │
                                                        ▼              ▼
                                              ┌──────────────────────────────┐
                                              │       READ_HEADER            │
                                              │   memcpy(&hdr, buf_, 20)     │
                                              └──────────────┬───────────────┘
                                                             │
                                              ┌──────────────┴───────────────┐
                                              │ version OK?  length <= max?  │
                                              └──────┬──────────┬────────────┘
                                                  no │          │ yes
                                                     ▼          ▼
                                          ┌─────────────┐  ┌──────────────────┐
                                          │ SKIP_MAGIC   │  │ WAIT_COMPLETE    │
                                          │ erase(0,2)   │  │ need 20+N+4     │
                                          │ resync++     │  │ bytes total?     │
                                          │ retry=true   │  └───┬─────┬───────┘
                                          └─────────────┘    no │     │ yes
                                                               ▼     ▼
                                                        ┌────────┐  ┌─────────────┐
                                                        │ WAIT   │  │ CRC_CHECK   │
                                                        │(return │  │ validate()  │
                                                        │ false) │  └──┬─────┬────┘
                                                        └────────┘  fail│     │pass
                                                                       ▼     ▼
                                                            ┌──────────┐  ┌──────────┐
                                                            │SKIP_MAGIC│  │ DISPATCH │
                                                            │crc_err++ │  │ type →   │
                                                            │resync++  │  │ lidar/   │
                                                            │retry     │  │ imu/cam/ │
                                                            └──────────┘  │ control  │
                                                                          │          │
                                                                          │packets++ │
                                                                          │erase(tot)│
                                                                          │retry=true│
                                                                          └──────────┘
```

### 5.2 Vendor UDP (datagram-framed, e.g., VelodyneVlp16Decoder)

```
              feed(data, len)     ← one recvfrom() = one call
                    │
                    ▼
        ┌───── len == 1206? ─────┐
        │ no                     │ yes
        ▼                        ▼
   ┌──────────┐       ┌──────────────────┐
   │ REJECT   │       │ VALIDATE_FLAG    │
   │malformed │       │ block[0].flag    │
   │   ++     │       │ == 0xFFEE?       │
   │ return   │       └────┬────────┬────┘
   └──────────┘          no│        │yes
                           ▼        ▼
                    ┌──────────┐  ┌────────────────────┐
                    │ REJECT   │  │ PARSE_12_BLOCKS    │
                    │malformed │  │ for b in 0..11:    │
                    │   ++     │  │   validate flag    │
                    │ return   │  │   read azimuth     │
                    └──────────┘  │   for ch in 0..15: │
                                  │     read dist+refl │
                                  │     skip if dist=0 │
                                  │     spherical→xyz  │
                                  │     scratch[n++]   │
                                  └────────┬───────────┘
                                           │
                                           ▼
                                  ┌────────────────────┐
                                  │ point_count > 0?   │
                                  └──┬────────────┬────┘
                                  no │            │ yes
                                     ▼            ▼
                              ┌──────────┐  ┌────────────────┐
                              │ DROP     │  │ EMIT_FRAME     │
                              │ (silent) │  │ make LidarFrame│
                              │ return   │  │ lidar_cb_()    │
                              └──────────┘  │ packets_parsed │
                                            │   ++           │
                                            └────────────────┘
```

**Key difference:** No accumulation, no resync loop, no CRC.  The entire
state machine is linear — one entry, one exit.  This is why vendor UDP
decoders can be allocation-free on the hot path.

---

## 6. Malformed Packet Handling

### 6.1 Categories of malformation

| Category | Detection | Recovery | Counter |
|----------|-----------|----------|---------|
| **Wrong size** (UDP) | `len != expected` | Drop entire datagram | `malformed_count` |
| **Bad magic / flag** | First bytes ≠ expected pattern | Stream: skip 2 bytes, rescan. Datagram: drop. | `resync_count` (stream), `malformed_count` (datagram) |
| **Version mismatch** | `hdr.version != expected` | Skip magic, rescan | `resync_count` |
| **Length overflow** | `payload_length > kMax` | Skip magic, rescan | `malformed_count` |
| **CRC failure** | `Crc32::validate() == false` | Skip magic, rescan | `checksum_errors` |
| **Truncated sub-payload** | `payload_len < sizeof(SubHeader)` | Drop packet (already CRC-validated, so just short) | `malformed_count` |
| **Out-of-range field values** | e.g. distance = 0 (no return), azimuth > 360° | Skip individual point/channel, don't drop packet | (silent — sensor-level, not an error) |
| **Sequence gap** | `seq != prev + 1` | Log, increment `packets_dropped`, continue | `packets_dropped` |

### 6.2 Zero-tolerance principle

**A malformed packet must never:**
- Cause a read past the input buffer (all field reads use `memcpy` to a local, not pointer cast + dereference)
- Cause an unbounded loop (accumulation buffer discards aggressively on bad state)
- Cause a crash or UB (every array index is bounds-checked against the buffer size)
- Silently corrupt downstream data (CRC/flag validation before dispatch)

### 6.3 Bounds-checking strategy

```cpp
// WRONG — aliasing UB + alignment UB + no bounds check:
auto* block = reinterpret_cast<const FiringBlock*>(pkt + offset);
float az = block->azimuth;  // UB if misaligned, UB if past buffer end

// CORRECT — safe memcpy with explicit bounds check:
if (offset + sizeof(uint16_t) > len) {
    ++stats_.malformed_count;
    return;
}
uint16_t raw_azimuth;
std::memcpy(&raw_azimuth, pkt + offset, sizeof(raw_azimuth));
```

Every field access in the vendor decoder follows this pattern.  The compiler
optimises `memcpy` of 2–8 bytes into a single load instruction — there is
zero runtime cost vs. the `reinterpret_cast` approach.

---

## 7. Integration with ConnectionManager

### 7.1 Current member

```cpp
// connection_manager.h — current:
PacketParser parser_;

// ConnectionManager::parser() returns PacketParser&
// HardwareDriver calls conn_->parser().on_lidar(cb)
```

### 7.2 Proposed change

```cpp
// connection_manager.h — proposed:
std::unique_ptr<IPacketDecoder> decoder_;

// Constructor default:
ConnectionManager(...) : decoder_(std::make_unique<PacketParser>()), ... { }

// New constructor overload:
ConnectionManager(std::unique_ptr<ITransport> transport,
                  std::unique_ptr<IPacketDecoder> decoder,  // injected
                  ConnectionConfig conn_cfg = {},
                  RetryConfig retry_cfg = {})

// Accessor:
IPacketDecoder& decoder() { return *decoder_; }

// HardwareDriver changes:
//   conn_->parser().on_lidar(cb)  →  conn_->decoder().on_lidar(cb)
//
// on_control() for Thunderbird-native:
//   if (auto* native = dynamic_cast<PacketParser*>(&conn_->decoder()))
//       native->on_control(ctrl_cb);
```

### 7.3 I/O loop change

```cpp
void io_loop() {
    constexpr size_t kBufSize = 64 * 1024;
    auto buf = std::make_unique<uint8_t[]>(kBufSize);

    while (io_running_) {
        if (!transport_->is_open()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // Phase 2: read_timestamped() when available (see PHASE2_TRANSPORT_DESIGN.md)
        size_t n = transport_->read(buf.get(), kBufSize, 100);
        if (n > 0) {
            decoder_->feed(buf.get(), n);   // ← polymorphic dispatch
        }
    }
}
```

---

## 8. No-Dynamic-Memory Guarantee — Where It Applies

| Layer | Heap allocation? | Reason |
|-------|-----------------|--------|
| `feed()` hot path (vendor UDP) | **No** | Parses in-place from recv buffer. Uses stack-resident `scratch_points_[]` array. |
| `feed()` hot path (Thunderbird native) | **Yes** — `buf_.insert()` | Stream framing requires accumulation. Can be improved with a ring buffer but not critical at current throughput. |
| SDK output construction | **Yes** — `make_shared<LidarFrame>()` + `points.assign()` | Unavoidable: SDK output types use `vector` and `shared_ptr`. This allocation happens once per frame, not per point. |
| `SequenceTracker` | **No** | Three integers, zero heap. |
| CRC computation | **No** | Table is `constexpr`, computation is in-place. |

The contract is: **the decoder's internal parsing logic does not allocate**.
The final conversion to SDK types does, because the SDK types are
designed for safe, ref-counted sharing across threads.

---

## 9. File Plan

| File | Type | Description |
|------|------|-------------|
| `sdk/include/thunderbird/packet_decoder.h` | New | `IPacketDecoder` + `DecoderStats` |
| `sdk/include/thunderbird/sequence_tracker.h` | New | `SequenceTracker` utility |
| `sdk/include/thunderbird/decoders/velodyne_vlp16.h` | New | VLP-16 decoder (header-only) |
| `sdk/include/thunderbird/decoders/decoder_factory.h` | New | `create_decoder(model_name)` factory |
| `sdk/include/thunderbird/packet_parser.h` | Modify | Inherit `IPacketDecoder`, use `host_timestamp()`, add sequence tracking |
| `sdk/include/thunderbird/connection_manager.h` | Modify | `PacketParser parser_` → `unique_ptr<IPacketDecoder> decoder_` |
| `sdk/include/thunderbird/drivers/hardware_driver.h` | Modify | `parser()` → `decoder()` |

### 9.1 Implementation order

1. `IPacketDecoder` + `DecoderStats` (new file, no dependencies)
2. `SequenceTracker` (new file, no dependencies)
3. `PacketParser` inherits `IPacketDecoder` (refactor, all existing tests must pass)
4. `ConnectionManager` uses `unique_ptr<IPacketDecoder>` (refactor, existing tests pass)
5. `VelodyneVlp16Decoder` (new, add unit test with captured packet data)
6. `decoder_factory.h` (new, trivial)
