# Thunderbird SDK — Phase 2 Transport Design

> **Status:** Design proposal  
> **Author:** Systems Architecture  
> **Date:** 2026-03-01  
> **Depends on:** `ITransport`, `EthernetTransport`, `PacketParser`, `ConnectionManager`, `types.h`  
> **Scope:** Two targeted enhancements — NOT a rewrite of the existing transport stack.

---

## 0. Why Not a New Transport Abstraction?

The existing transport stack is well-layered and sound:

```
DeviceManager
  └── HardwareDriver
        └── ConnectionManager   (retry, reconnect, heartbeat, I/O thread)
              ├── ITransport     (byte-level: open/close/read/write)
              │     ├── EthernetTransport  (TCP control + UDP data, poll()-based)
              │     ├── UsbTransport       (libusb stub, complete API)
              │     └── SimulatedTransport (in-memory FIFO for testing)
              └── PacketParser   (incremental state machine → LidarFrame / ImuSample / CameraFrame)
```

The user-facing requirements (UDP, USB, non-blocking, high throughput) are already
addressed by this architecture.  A new transport abstraction would either duplicate
or displace code that works.

**This document designs the two real gaps:**

1. **SO_TIMESTAMPING** — recover NIC-level RX timestamps for clock-offset estimation.
2. **VendorPacketDecoder** — support real-hardware packet formats (Velodyne, Ouster,
   Livox, etc.) that don't speak the Thunderbird wire protocol.

---

## 1. SO_TIMESTAMPING — NIC-Level RX Timestamps

### 1.1 Problem

The `PacketParser` currently stamps all decoded frames with `Timestamp::now()`
at the moment `dispatch()` runs (see `packet_parser.h:192`):

```cpp
Timestamp host_ts = Timestamp::now();   // ← steady_clock::now() in userspace
```

This host timestamp has **microsecond-scale jitter** from thread scheduling,
`poll()` wake-up latency, and buffer copies.  For PTP-grade clock-offset
estimation (Phase 2 roadmap item), we need the kernel or NIC RX timestamp,
which captures *when the packet actually arrived at the network interface* —
typically sub-microsecond precision.

### 1.2 Design

#### 1.2.1 Extend `ITransport` with `read_timestamped()`

Add a non-breaking virtual method with a default implementation:

```cpp
// transport.h

/// Result of a timestamped read operation.
struct ReadResult {
    size_t  bytes_read{0};
    int64_t rx_timestamp_ns{0};   ///< Kernel/NIC RX timestamp (ns since epoch).
                                  ///< 0 = not available (transport doesn't support it).
};

class ITransport {
public:
    // ... existing pure virtuals unchanged ...

    /// Read with optional kernel/NIC RX timestamp.
    /// Default implementation delegates to read() and returns no timestamp.
    virtual ReadResult read_timestamped(uint8_t* buf, size_t max_bytes,
                                        uint32_t timeout_ms) {
        ReadResult r;
        r.bytes_read = read(buf, max_bytes, timeout_ms);
        return r;
    }
};
```

**Backward compatibility:** `SimulatedTransport` and `UsbTransport` inherit the
default and work unchanged.  Only `EthernetTransport` overrides.

#### 1.2.2 Override in `EthernetTransport`

On Linux, `EthernetTransport::open()` enables kernel timestamping on the
UDP socket:

```cpp
// In open(), after binding the UDP socket:
#ifdef __linux__
int flags = SOF_TIMESTAMPING_RX_SOFTWARE
          | SOF_TIMESTAMPING_SOFTWARE;     // minimum: kernel RX timestamp
// If NIC supports hardware timestamping, also request:
// flags |= SOF_TIMESTAMPING_RX_HARDWARE | SOF_TIMESTAMPING_RAW_HARDWARE;
setsockopt(udp_sock_, SOL_SOCKET, SO_TIMESTAMPING, &flags, sizeof(flags));
#endif
```

`read_timestamped()` uses `recvmsg()` instead of `recvfrom()` to access
ancillary data:

```cpp
ReadResult read_timestamped(uint8_t* buf, size_t max_bytes,
                            uint32_t timeout_ms) override
{
    if (!open_) return {};

    // poll() as before ...
    // (same poll()/select() logic already in read())

    ReadResult result;

#ifdef __linux__
    if (/* UDP ready */) {
        struct msghdr msg{};
        struct iovec  iov;
        iov.iov_base = buf;
        iov.iov_len  = max_bytes;
        msg.msg_iov    = &iov;
        msg.msg_iovlen = 1;

        // Ancillary data buffer for SCM_TIMESTAMPING
        alignas(cmsghdr) char ctrl[256];
        msg.msg_control    = ctrl;
        msg.msg_controllen = sizeof(ctrl);

        ssize_t n = ::recvmsg(udp_sock_, &msg, 0);
        if (n > 0) {
            result.bytes_read = static_cast<size_t>(n);

            // Extract kernel RX timestamp from cmsg
            for (cmsghdr* cm = CMSG_FIRSTHDR(&msg); cm;
                 cm = CMSG_NXTHDR(&msg, cm)) {
                if (cm->cmsg_level == SOL_SOCKET &&
                    cm->cmsg_type  == SO_TIMESTAMPING) {
                    // Array of 3 timespecs: [software, deprecated, hardware]
                    auto* ts = reinterpret_cast<const timespec*>(CMSG_DATA(cm));
                    // Prefer hardware (index 2), fall back to software (index 0)
                    const timespec& chosen = (ts[2].tv_sec || ts[2].tv_nsec)
                                           ? ts[2] : ts[0];
                    result.rx_timestamp_ns = chosen.tv_sec * 1'000'000'000LL
                                           + chosen.tv_nsec;
                    break;
                }
            }
        }
    }
    // TCP fallback unchanged (no kernel timestamp for TCP control)
    if (result.bytes_read == 0) {
        result.bytes_read = /* recv() on TCP, same as current read() */;
        // rx_timestamp_ns stays 0 — TCP control doesn't need PTP precision
    }
#else
    // Windows / other: delegate to read(), no kernel timestamp
    result.bytes_read = read(buf, max_bytes, timeout_ms);
#endif

    return result;
}
```

On **Windows**: `SO_TIMESTAMPING` is not available.  The default fallback
(no kernel timestamp) is appropriate — Windows deployments can use
`QueryPerformanceCounter` at the application level, which is sufficient
for the software-PTP accuracy tier Windows supports.

#### 1.2.3 Propagate through ConnectionManager I/O loop

`ConnectionManager::io_loop()` switches from `read()` to `read_timestamped()`:

```cpp
void io_loop() {
    constexpr size_t kBufSize = 64 * 1024;
    auto buf = std::make_unique<uint8_t[]>(kBufSize);

    while (io_running_) {
        if (!transport_->is_open()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        auto result = transport_->read_timestamped(buf.get(), kBufSize, 100);
        if (result.bytes_read > 0) {
            // If kernel timestamp available, inject into decoder
            // so decoded frames get precise host_timestamp.
            if (result.rx_timestamp_ns != 0) {
                decoder_->set_rx_timestamp(result.rx_timestamp_ns);
            }
            decoder_->feed(buf.get(), result.bytes_read);
        }
    }
}
```

#### 1.2.4 Timestamp flow diagram

```
  NIC hardware clock
        │
        ▼
  ┌──────────────┐
  │ SO_TIMESTAMPING│  ← kernel captures RX timestamp at interrupt time
  └──────┬───────┘
         │ cmsg → rx_timestamp_ns
         ▼
  ┌──────────────┐
  │ read_         │
  │ timestamped() │  ← EthernetTransport extracts from recvmsg() ancillary data
  └──────┬───────┘
         │ ReadResult { bytes, rx_ts }
         ▼
  ┌──────────────┐
  │ I/O loop     │  ← ConnectionManager calls set_rx_timestamp() then feed()
  └──────┬───────┘
         │
         ▼
  ┌──────────────┐
  │ Decoder      │  ← Uses rx_ts as host_timestamp on decoded frames
  │ (IPacket-    │     (instead of steady_clock::now())
  │  Decoder)    │
  └──────┬───────┘
         │ LidarFrame.host_timestamp = rx_ts
         ▼
  ┌──────────────┐
  │ Clock offset │  ← Pairs (hw_timestamp, host_timestamp) for linear regression
  │ estimator    │
  └──────────────┘
```

### 1.3 Why not SO_RCVBUF / recvmmsg / epoll?

| Optimization | When it matters | Status |
|---|---|---|
| `SO_RCVBUF` increase | When kernel drops UDP packets under burst load | Add as a tuning knob in `EthernetTransport::open()` — not an architecture change |
| `recvmmsg()` | When syscall overhead dominates (>500k small packets/s) | At 100 Mbps with ~1200-byte UDP datagrams, that's ~10k packets/s — `recvmsg()` overhead is negligible |
| `epoll` | When multiplexing >100 fds | We have 2 sockets per device. `poll()` is equally efficient for 2 fds |

These are **tuning knobs** to add later behind the existing API if profiling shows need.
They do not require design changes.

---

## 2. VendorPacketDecoder — Real-Hardware Protocol Support

### 2.1 Problem

The existing `PacketParser` assumes the Thunderbird wire protocol (`0xBEEF` magic,
20-byte header, CRC-32).  Real LiDAR/IMU/camera hardware uses vendor-proprietary
UDP packet formats:

| Vendor | Format | Typical packet size |
|--------|--------|-------------------|
| Velodyne VLP-16 | 12 firing blocks × 32 channels, factory bytes, GPS timestamp | 1206 bytes |
| Ouster OS1 | Lidar & IMU on separate UDP ports, column-major layout | 12608 bytes (lidar) |
| Livox Mid-360 | Dual-return, Cartesian/spherical selectable, custom header | Variable |
| Hesai Pandar | Multi-return, wedge-based firing sequence | ~1100 bytes |

None of these speak the Thunderbird protocol.  The `PacketParser` cannot decode them.

### 2.2 Design

#### 2.2.1 `IPacketDecoder` interface

A new abstraction that generalizes what `PacketParser` does:

```cpp
// packet_decoder.h
#pragma once

#include "thunderbird/types.h"
#include <cstdint>
#include <string>

namespace thunderbird {

/// Abstract decoder: converts raw transport bytes into typed SDK objects.
///
/// Implementations:
///   • PacketParser         — Thunderbird native wire protocol (existing)
///   • VelodyneVlp16Decoder — Velodyne VLP-16 UDP format
///   • OusterOs1Decoder     — Ouster OS1/OS2 UDP format
///   • LivoxMid360Decoder   — Livox Mid-360 point cloud format
///
/// Each decoder is a streaming state machine: bytes are fed incrementally
/// via feed(), and complete sensor frames are dispatched to registered
/// callbacks.  This is the same contract as PacketParser, but without
/// assuming any specific wire format.
class IPacketDecoder {
public:
    virtual ~IPacketDecoder() = default;

    // ── Callback registration ───────────────────────────────────────────

    void on_lidar(LidarCallback cb)    { lidar_cb_  = std::move(cb); }
    void on_imu(ImuCallback cb)        { imu_cb_    = std::move(cb); }
    void on_camera(CameraCallback cb)  { camera_cb_ = std::move(cb); }

    // ── Ingestion ───────────────────────────────────────────────────────

    /// Feed raw bytes from the transport.  Implementations parse and
    /// dispatch complete frames via the registered callbacks.
    virtual void feed(const uint8_t* data, size_t len) = 0;

    /// Reset internal parser state (e.g. after reconnect).
    virtual void reset() = 0;

    // ── RX timestamp injection ──────────────────────────────────────────

    /// Set the kernel/NIC RX timestamp for the NEXT feed() call.
    /// Decoders use this as host_timestamp on produced frames instead
    /// of steady_clock::now().  Set to 0 to revert to software timestamps.
    void set_rx_timestamp(int64_t rx_ns) { rx_timestamp_ns_ = rx_ns; }

    // ── Metadata ────────────────────────────────────────────────────────

    /// Human-readable decoder name (e.g. "Velodyne VLP-16", "Thunderbird native").
    virtual std::string decoder_name() const = 0;

protected:
    /// Subclasses use this to get the best available host timestamp.
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

**Key design choices:**

- **`host_timestamp()` helper**: centralizes the SO_TIMESTAMPING → fallback logic.
  Every decoder calls this instead of `Timestamp::now()` directly.
- **Callbacks in base class**: identical signature to existing `PacketParser`,
  so `ConnectionManager` doesn't need to know about callback registration.
- **`set_rx_timestamp()`**: called by the I/O loop before `feed()`, avoids
  polluting the `feed()` signature with timestamp parameters.  This is safe
  because the I/O loop is single-threaded.
- **No control callback in interface**: vendor protocols don't have Thunderbird
  control messages.  `PacketParser` keeps its `on_control()` as an extension.

#### 2.2.2 Make `PacketParser` implement `IPacketDecoder`

```cpp
// Minimal change: PacketParser inherits IPacketDecoder
class PacketParser : public IPacketDecoder {
public:
    PacketParser() { buf_.reserve(64 * 1024); }

    // IPacketDecoder interface
    void feed(const uint8_t* data, size_t len) override;
    void reset() override;
    std::string decoder_name() const override { return "Thunderbird native"; }

    // Extension: control/handshake callback (not in IPacketDecoder)
    void on_control(ControlCallback cb) { control_cb_ = std::move(cb); }

    const ParserStats& stats() const { return stats_; }

    // ... rest unchanged ...
};
```

The existing `dispatch()` method changes one line:

```cpp
// Before:
Timestamp host_ts = Timestamp::now();

// After:
Timestamp host_ts = host_timestamp();   // uses SO_TIMESTAMPING if available
```

#### 2.2.3 Example: VelodyneVlp16Decoder (sketch)

```cpp
class VelodyneVlp16Decoder final : public IPacketDecoder {
public:
    void feed(const uint8_t* data, size_t len) override {
        // Velodyne VLP-16 packets are always exactly 1206 bytes.
        // No framing needed — each UDP datagram = one packet.
        if (len != 1206) return;

        // Parse 12 firing blocks × 2 returns × 16 channels = 384 points
        // Extract azimuth, distance, intensity per point
        // Convert to LidarFrame with SDK coordinate convention
        // Set frame->host_timestamp = host_timestamp()
        // Set frame->timestamp from the 4-byte GPS timestamp at offset 1200
        // Fire lidar_cb_
    }

    void reset() override { /* stateless — no accumulation buffer */ }

    std::string decoder_name() const override { return "Velodyne VLP-16"; }
};
```

**Note:** Velodyne packets are self-framing (each UDP datagram is a complete
packet), so no accumulation buffer is needed — unlike the Thunderbird protocol
which can split across reads.  This is why `feed()` is per-decoder: each vendor's
framing model is different.

#### 2.2.4 Decoder factory

```cpp
// decoder_factory.h

/// Create the appropriate decoder for a device model.
/// model_name comes from DeviceInfo (handshake) or user configuration.
///
/// Known models:
///   "thunderbird"     → PacketParser (native protocol)
///   "vlp16"           → VelodyneVlp16Decoder
///   "os1-64"          → OusterOs1Decoder
///   "mid360"          → LivoxMid360Decoder
///
/// Returns nullptr for unknown models.
std::unique_ptr<IPacketDecoder> create_decoder(const std::string& model_name);
```

#### 2.2.5 Integration with ConnectionManager

`ConnectionManager` currently owns a `PacketParser` directly:

```cpp
// Current (connection_manager.h):
PacketParser parser_;
```

Change to:

```cpp
// Proposed:
std::unique_ptr<IPacketDecoder> decoder_;

// For backward compat, PacketParser is the default:
// Constructor creates a PacketParser by default.
// New constructor overload accepts a custom decoder.
```

The I/O loop changes from:

```cpp
parser_.feed(buf.get(), n);
```

to:

```cpp
decoder_->feed(buf.get(), n);
```

**Control-message handling for Thunderbird-native devices:** When the decoder
is a `PacketParser`, `ConnectionManager` can still register its control callback
via a downcast (or better, via a separate method that checks `decoder_name()`).
For vendor devices, the control path is different anyway — most LiDARs start
streaming immediately on socket bind, no handshake needed.

### 2.3 ConnectionManager adaptations for vendor devices

Vendor LiDARs typically:
1. Start streaming as soon as the host binds to their data port (no handshake)
2. Send on a fixed multicast or unicast UDP port (no TCP control channel)
3. Don't support heartbeat (link loss = no packets for N seconds)

This means `ConnectionManager`'s `ConnectionStateMachine` (handshake,
heartbeat, StartStream/StopStream) is irrelevant for vendor devices.

**Proposed approach:**

```cpp
struct ConnectionManagerConfig {
    // ... existing RetryConfig, ConnectionConfig ...

    /// When true, skip protocol handshake and heartbeat.
    /// The device is assumed streaming as soon as transport is open.
    /// Link health is monitored by packet-timeout instead of heartbeat.
    bool raw_streaming_mode{false};

    /// In raw_streaming_mode, how long without any received packet
    /// before declaring link loss (ms).
    uint32_t packet_timeout_ms{2000};
};
```

When `raw_streaming_mode` is true:
- `connect()` only calls `transport_->open()` (no handshake exchange)
- `start_streaming()` just starts the I/O thread (no StartStream command)
- Link health is monitored by tracking time since last `feed()` produced a callback

This avoids forking `ConnectionManager` into two classes.

---

## 3. Full Integration Diagram

```
                          ┌──────────────────────────────────────┐
                          │         DeviceManager                │
                          │                                      │
                          │  config.model = "vlp16"              │
                          │  config.uri   = "eth://192.168.1.201:2368" │
                          └──────────────┬───────────────────────┘
                                         │
                            create_decoder("vlp16")
                            create transport from URI
                                         │
                                         ▼
                          ┌──────────────────────────────────────┐
                          │      ConnectionManager               │
                          │                                      │
                          │  decoder_: VelodyneVlp16Decoder      │
                          │  transport_: EthernetTransport       │
                          │  raw_streaming_mode: true            │
                          │                                      │
                          │  ┌───────── I/O thread ───────────┐  │
                          │  │                                 │  │
                          │  │  result = transport_->           │  │
                          │  │      read_timestamped(buf, ...)  │  │
                          │  │                                 │  │
                          │  │  decoder_->set_rx_timestamp(    │  │
                          │  │      result.rx_timestamp_ns)    │  │
                          │  │                                 │  │
                          │  │  decoder_->feed(buf, n)         │  │
                          │  │    │                            │  │
                          │  │    ├─→ lidar_cb_(frame)         │  │
                          │  │    ├─→ imu_cb_(sample)          │  │
                          │  │    └─→ camera_cb_(frame)        │  │
                          │  │                                 │  │
                          │  └─────────────────────────────────┘  │
                          └──────────────────────────────────────┘

  frame->timestamp      = device HW clock (extracted from vendor payload)
  frame->host_timestamp = NIC RX timestamp (from SO_TIMESTAMPING)
                         OR steady_clock::now() (fallback)
```

---

## 4. Clock-Offset Estimation (downstream consumer)

This design enables the Phase 2 roadmap item "Clock-offset estimation (linear
regression on HW↔host timestamps)" without any further transport changes:

```
For each decoded frame:
    hw_ts  = frame->timestamp.nanoseconds       ← from device payload
    rx_ts  = frame->host_timestamp.nanoseconds   ← from SO_TIMESTAMPING

    offset_sample = rx_ts - hw_ts

    // Feed into existing drift estimator (SlamTimeSyncEngine / TimeSyncEngine)
    // Linear regression over sliding window → estimated clock offset
```

The existing `SlamTimeSyncConfig::ClockDomain` enum already supports selecting
Hardware vs. Host timestamps.  The improvement is that `Host` timestamps
are now **kernel-quality** (sub-μs) instead of userspace-quality (10–100 μs jitter).

---

## 5. File Changes Summary

| File | Change | Breaking? |
|------|--------|-----------|
| `transport.h` | Add `ReadResult` struct and `read_timestamped()` default virtual | No |
| `ethernet_transport.h` | Override `read_timestamped()` with SO_TIMESTAMPING (Linux) | No |
| `ethernet_transport.h` | Add `SO_RCVBUF` sizing in `open()` (tuning, optional) | No |
| `packet_decoder.h` | **New file** — `IPacketDecoder` interface | N/A |
| `packet_parser.h` | Inherit `IPacketDecoder`, use `host_timestamp()` helper | No |
| `connection_manager.h` | Own `unique_ptr<IPacketDecoder>` instead of `PacketParser` member | Minimal* |
| `connection_manager.h` | Add `raw_streaming_mode` to config | No |
| `connection_manager.h` | `io_loop()` uses `read_timestamped()` + `set_rx_timestamp()` | No |
| `device_manager.cpp` | Pass model name → `create_decoder()`, inject into ConnectionManager | No |
| `decoder_factory.h` | **New file** — `create_decoder()` factory function | N/A |
| vendor decoders | **New files** — one per supported vendor (implemented as needed) | N/A |

*`ConnectionManager::parser()` currently returns `PacketParser&`.  This would need
to become `IPacketDecoder&` (or be deprecated in favor of directly registering
callbacks).  Since `HardwareDriver` only uses `parser()` to register sensor
callbacks, and those are on `IPacketDecoder`, this is a minor signature change.

---

## 6. What This Design Does NOT Include

| Item | Reason |
|------|--------|
| epoll replacement for poll() | Not beneficial for 2 file descriptors |
| Zero-copy kernel bypass (io_uring, DPDK) | Overkill at 100 Mbps; revisit if profiling shows syscall bottleneck |
| New `ITransport` subclasses | Ethernet and USB already exist and are correct |
| recvmmsg() batching | ~10k packets/s doesn't saturate single recvmsg(); add later if profiling warrants |
| Multi-device multiplexing | Current design is one ConnectionManager per device; scales fine for 2–4 devices |

---

## 7. Implementation Order

1. **`IPacketDecoder`** interface + make `PacketParser` implement it (pure refactor, tests pass)
2. **`ConnectionManager` update** — accept `IPacketDecoder*`, add `raw_streaming_mode`
3. **`read_timestamped()` in `ITransport`** — default impl + `EthernetTransport` override
4. **I/O loop integration** — wire `read_timestamped()` → `set_rx_timestamp()` → `feed()`
5. **First vendor decoder** — e.g., `VelodyneVlp16Decoder` (when hardware is available for testing)
6. **Clock-offset estimation** — feed (hw_ts, rx_ts) pairs into drift estimator
