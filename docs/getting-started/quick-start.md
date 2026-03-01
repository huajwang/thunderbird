# Quick Start Guide

Get the Thunderbird SDK running in under 5 minutes — no hardware required.

---

## Prerequisites

| Requirement | Minimum | Recommended |
|-------------|---------|-------------|
| C++ compiler | GCC 9 / Clang 10 / MSVC 2019 | GCC 12+ / Clang 15+ |
| CMake | 3.16 | 3.22+ |
| OS | Linux (x86_64, aarch64), macOS, Windows (Cygwin/MSYS2) | Ubuntu 22.04 |

Optional:
- **Python 3.8+** + `pybind11` — for Python bindings
- **Doxygen** + **Graphviz** — for API docs generation
- **ROS 2 Humble/Iron** — for the ROS 2 bridge node

---

## 1. Clone and Build

```bash
git clone https://github.com/thunderbird-sdk/thunderbird.git
cd thunderbird

cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

The SDK builds in **simulated mode** by default — three software-driven sensor
generators produce realistic LiDAR, IMU, and Camera data at configurable rates.
No physical hardware is needed.

## 2. Run the Tests

```bash
cd build
ctest --output-on-failure
```

All 18 tests should pass.  If any fail, see the
[Troubleshooting Guide](../troubleshooting/troubleshooting.md).

## 3. Run Your First Example

```bash
./build/examples/basic_streaming
```

Expected output:
```
LiDAR frame: 1024 points @ t=1000000000
IMU sample: accel=[0.01, -9.81, 0.02] @ t=1005000000
Camera frame: 640x480 RGB @ t=1033333333
Sync bundle #1 produced
...
Streamed for 2 seconds — stopping.
```

## 4. Try the Synchronized Data API

```bash
./build/examples/sync_layer_demo
```

This demonstrates the time-synchronization layer:
- **Pull API** — poll for the latest `SyncedFrame` on your own schedule
- **Callback API** — receive `SyncedFrame` events as they become ready

## 5. Write Your First Program

Create `my_app.cpp`:

```cpp
#include <thunderbird/device_manager.h>
#include <cstdio>
#include <thread>

int main() {
    // Simulated mode — no URI needed.
    thunderbird::DeviceManager device;

    // Register a LiDAR callback.
    device.on_lidar([](auto frame) {
        std::printf("Got %zu points\n", frame->points.size());
    });

    // Connect → start → run for 3 seconds → stop.
    if (device.connect() != thunderbird::Status::OK) {
        std::fprintf(stderr, "Connection failed\n");
        return 1;
    }

    device.start();
    std::this_thread::sleep_for(std::chrono::seconds(3));
    device.stop();
    device.disconnect();

    std::printf("Done — %llu sync bundles produced\n",
                static_cast<unsigned long long>(device.sync_bundles_produced()));
    return 0;
}
```

Build it against the SDK:

```bash
g++ -std=c++20 my_app.cpp \
    -I thunderbird/sdk/include \
    -I thunderbird/build/sdk/include \
    -L thunderbird/build/sdk \
    -lthunderbird_sdk -lpthread \
    -o my_app

./my_app
```

Or add it to the CMake build:

```cmake
add_executable(my_app my_app.cpp)
target_link_libraries(my_app PRIVATE thunderbird_sdk)
```

---

## Next Steps

| What | Where |
|------|-------|
| Connect to real hardware | [Hardware Setup Guide](hardware-setup.md) |
| Understand time alignment | [Time Synchronization Guide](../guides/time-synchronization.md) |
| Record and replay data | [Recording & Playback Guide](../guides/recording-playback.md) |
| Tune perception pipeline | [Perception Pipeline Guide](../guides/perception-pipeline.md) |
| Use Python bindings | [examples/python_example.py](../../examples/python_example.py) |
| Full API reference | [API Docs](../api/overview.md) |
