# Callback Threading Guide

How sensor callbacks interact with the I/O thread and strategies for
offloading heavy processing without stalling data ingestion.

---

## Overview

The Thunderbird SDK delivers sensor data (LiDAR, IMU, Camera) via
**synchronous callbacks** that fire on the I/O thread inside
`IPacketDecoder::feed()`.  This means:

- Callbacks block the I/O thread until they return.
- While a callback is running, no new bytes are read from the transport.
- All three sensor callbacks (`on_lidar`, `on_imu`, `on_camera`) share
  the same I/O thread.

```
Device ──packets──▶ ITransport::read_timestamped()
                          │
                    io_loop() thread
                          │
                    decoder_->feed(buf, len)
                          │
                    ┌─────┴─────┐
                    ▼           ▼
              lidar_cb_()   imu_cb_()   ◀── your callbacks run here
                    │           │
                    └─────┬─────┘
                          │
                    read next chunk
```

---

## When Inline Processing Is Fine

If your callback is fast (sub-millisecond), running directly on the I/O
thread is the simplest and lowest-latency option:

```cpp
std::atomic<int> count{0};

conn_mgr->decoder().on_lidar([&](std::shared_ptr<const LidarFrame> f) {
    ++count;                                    // atomic increment
    latest_frame.store(f);                      // publish to another thread
});
```

Good candidates for inline callbacks:

- Incrementing counters / updating statistics
- Storing the latest frame in an `std::atomic<std::shared_ptr<…>>`
- Logging (if your logger is non-blocking)
- Lightweight filtering or field extraction

---

## When to Offload to a Worker Thread

If your callback takes more than ~1 ms, it risks stalling the I/O
pipeline.  Symptoms include:

- **Packet drops** — the OS socket receive buffer overflows because the
  reader isn't draining it fast enough.
- **Increased latency** — frames queue up behind slow callback execution.
- **Missed heartbeats** — if the I/O thread is also feeding heartbeat
  replies to the parser, delays can cause the connection state machine
  to declare link loss.

Common heavyweight operations:

- Point cloud downsampling, voxel filtering, or ground segmentation
- SLAM / localisation updates
- Writing to disk or network
- GPU upload / inference

### Producer–Consumer Pattern

Push frames into a thread-safe queue and process them on a dedicated
worker thread.  The `shared_ptr<const T>` parameter type makes
enqueuing cheap (reference-count bump, no deep copy):

```cpp
#include <thunderbird/sensor_queue.h>   // or your own concurrent queue

thunderbird::SensorQueue<std::shared_ptr<const LidarFrame>> lidar_q(64);

// Register a lightweight callback that just enqueues.
conn_mgr->decoder().on_lidar([&](std::shared_ptr<const LidarFrame> f) {
    lidar_q.push(f);   // fast — ref-count bump only
});

// Worker thread — processes frames at its own pace.
std::thread worker([&] {
    while (running) {
        auto frame = lidar_q.pop_for(std::chrono::milliseconds(100));
        if (frame) {
            heavy_processing(*frame);   // doesn't stall I/O
        }
    }
});
```

### Using the Pull API

The SDK also provides a built-in pull (polling) API via the
`DataAbstractionLayer`, which internally uses the same queue pattern.
This is useful when your application loop drives the cadence:

```cpp
auto& layer = dev.data_layer();

while (running) {
    if (auto frame = layer.getNextLidarFrame()) {
        process(*frame);
    }
    if (auto imu = layer.waitForImuFrame(std::chrono::milliseconds(10))) {
        propagate(*imu);
    }
}
```

See [pull_api_demo.cpp](../../examples/pull_api_demo.cpp) for a
complete example.

---

## SPSC Ring Buffer for High-Throughput Paths

For maximum throughput with a single producer and single consumer, use the
lock-free `RingBuffer` (SPSC) provided by the SDK.  It avoids mutex
contention entirely and drops the oldest entry on overflow:

```cpp
thunderbird::RingBuffer<std::shared_ptr<const LidarFrame>, 32> ring;

conn_mgr->decoder().on_lidar([&](std::shared_ptr<const LidarFrame> f) {
    ring.push(f);   // lock-free, overwrites oldest if full
});
```

This is the same pattern used internally by the perception pipeline
(see [PERCEPTION_LAYER_DESIGN.md](../PERCEPTION_LAYER_DESIGN.md)).

---

## Thread Safety Summary

| Operation | Thread | Safe? |
|---|---|---|
| `on_lidar()` / `on_imu()` / `on_camera()` | Any (before streaming) | Yes — but not thread-safe to call *during* `feed()` |
| Callback execution | I/O thread | Single-threaded — no locking needed inside the callback |
| `on_event()` | Any | Yes — mutex-protected in `ConnectionManager` |
| `shared_ptr<const T>` copies | Any | Yes — atomic ref-count |

---

## See Also

- [comm_layer_demo.cpp](../../examples/comm_layer_demo.cpp) — inline
  callback example
- [pull_api_demo.cpp](../../examples/pull_api_demo.cpp) — pull API
  example
- [VENDOR_PARSER_DESIGN.md](../VENDOR_PARSER_DESIGN.md) — decoder
  contract and threading model
- [PERCEPTION_LAYER_DESIGN.md](../PERCEPTION_LAYER_DESIGN.md) — SPSC
  ring buffer usage in the perception pipeline
