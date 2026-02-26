# Thunderbird SDK — Multi-Sensor Device SDK (PoC)

A C++17 SDK for interfacing with a fused **LiDAR + IMU + Camera** sensor device.
This PoC focuses on **SDK infrastructure, data flow, and time synchronization** —
no AI/perception algorithms are included yet.

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                        User Application                             │
│   on_lidar()  on_imu()  on_camera()  on_sync()                     │
└──────────┬──────────┬──────────┬──────────┬────────────────────────┘
           │          │          │          │
┌──────────▼──────────▼──────────▼──────────▼────────────────────────┐
│                      DeviceManager (public API)                     │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐   ┌───────────────────┐ │
│  │ LiDAR    │  │ IMU      │  │ Camera   │   │   SyncEngine      │ │
│  │ Driver   │  │ Driver   │  │ Driver   │──▶│ (nearest-neighbour│ │
│  │ (thread) │  │ (thread) │  │ (thread) │   │  time alignment)  │ │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘   └─────────┬─────────┘ │
│       │              │             │                    │           │
│  ┌────▼──────────────▼─────────────▼───┐     ┌─────────▼─────────┐ │
│  │       ITransport (abstract)         │     │   SyncBundle      │ │
│  │  SimulatedTransport │ USB │ Ethernet│     │ (LiDAR+IMU+Cam)  │ │
│  └─────────────────────────────────────┘     └───────────────────┘ │
└─────────────────────────────────────────────────────────────────────┘
```

### Key design decisions

| Decision | Rationale |
|---|---|
| **`shared_ptr<const T>` for all frames** | Zero-copy fan-out to user callbacks + sync engine; immutable after creation |
| **One thread per sensor driver** | Matches real hardware (independent I/O rates); avoids head-of-line blocking |
| **Lock-free `RingBuffer`** | SPSC pattern for high-throughput paths; drops oldest on overflow |
| **Nearest-neighbour sync** | Simple, deterministic, configurable tolerance — good enough for PoC |
| **PImpl in `DeviceManager`** | Stable ABI; hides internal headers from consumers |

---

## Repository Layout

```
thunderbird/
├── CMakeLists.txt          # Top-level build (version source-of-truth)
├── pyproject.toml          # Python packaging (scikit-build-core)
├── sdk/                    # Core SDK (headers, sources, CMake config)
├── examples/               # C++ example programs
├── tests/                  # Unit tests (CTest)
├── python/                 # pybind11 bindings + Python example
├── ros2_bridge/            # ROS 2 node
├── debian/                 # Debian packaging
├── docker/                 # Dockerfiles (runtime + dev)
├── scripts/                # Version extraction, changelog, CI helpers
├── .github/workflows/      # CI, Release, Promote, Security workflows
├── ENVIRONMENTS.md         # Environment protection rules
└── SECURITY.md             # Vulnerability reporting policy
```

> **Full tree** with per-file descriptions is in
> [`.github/copilot-instructions.md`](.github/copilot-instructions.md).

---

## Quick Start

### Prerequisites

- **C++17** compiler (GCC 9+, Clang 10+, MSVC 2019+)
- **CMake** ≥ 3.16
- *(Optional)* **pybind11** for Python bindings
- *(Optional)* **ROS 2 Humble/Iron** for the bridge node

### Build (simulated mode — no hardware needed)

```bash
# Clone
git clone <repo-url> thunderbird && cd thunderbird

# Configure + build
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel

# Run tests
cd build && ctest --output-on-failure
```

### Run examples

```bash
./build/examples/basic_streaming
./build/examples/sync_demo
./build/examples/raw_sensors
```

### Build with Python bindings

```bash
pip install pybind11
cmake -B build -DTHUNDERBIRD_BUILD_PYTHON=ON
cmake --build build --parallel

# Run
cd python
PYTHONPATH=../build/python python example.py
```

### Build the ROS 2 bridge

```bash
# Inside a ROS 2 workspace
ln -s /path/to/thunderbird/ros2_bridge src/thunderbird_ros2_bridge
colcon build --packages-select thunderbird_ros2_bridge
source install/setup.bash
ros2 run thunderbird_ros2_bridge thunderbird_ros2_node
```

---

## C++ API Reference

### `DeviceManager` — primary entry point

```cpp
#include <thunderbird/thunderbird.h>

thunderbird::DeviceConfig cfg;
cfg.lidar_hz   = 10.0;   // LiDAR scan rate
cfg.imu_hz     = 200.0;  // IMU sample rate
cfg.camera_fps = 30.0;   // Camera frame rate

thunderbird::DeviceManager device(cfg);

// Register callbacks BEFORE start()
device.on_lidar([](std::shared_ptr<const thunderbird::LidarFrame> f) {
    // f->points, f->timestamp, f->sequence_number
});

device.on_imu([](std::shared_ptr<const thunderbird::ImuSample> s) {
    // s->accel, s->gyro, s->temperature, s->timestamp
});

device.on_camera([](std::shared_ptr<const thunderbird::CameraFrame> f) {
    // f->data (raw pixels), f->width, f->height, f->format
});

device.on_sync([](std::shared_ptr<const thunderbird::SyncBundle> b) {
    // b->lidar, b->imu, b->camera — time-aligned
});

device.connect();      // → Status::OK
device.start();        // begins streaming on sensor threads

// ... your processing loop ...

device.stop();
device.disconnect();
```

### Data types

| Type | Fields | Notes |
|---|---|---|
| `Timestamp` | `int64_t nanoseconds` | Steady-clock based; `.to_seconds()` helper |
| `LidarFrame` | `timestamp`, `points[]`, `sequence_number` | Each point: x/y/z/intensity/ring |
| `ImuSample` | `timestamp`, `accel[3]`, `gyro[3]`, `temperature` | SI units (m/s², rad/s, °C) |
| `CameraFrame` | `timestamp`, `width`, `height`, `format`, `data[]` | Raw pixel buffer |
| `SyncBundle` | `reference_time`, `lidar`, `imu`, `camera` | Any member may be `nullptr` |

### Sync configuration

```cpp
thunderbird::SyncConfig sync;
sync.tolerance_ns    = 50'000'000;  // 50 ms — max drift between sensors
sync.poll_interval_ms = 5;          // sync thread polling period
sync.reference_sensor = thunderbird::SensorType::LiDAR;

cfg.sync = sync;
```

---

## Time Synchronization — How It Works

```
Time ──────────────────────────────────────────────────────▶

LiDAR  ─── L1 ─────────────── L2 ─────────────── L3 ───
            10 Hz

IMU    ─ I1 I2 I3 I4 I5 I6 I7 I8 I9 I10 I11 I12 I13 ──
            200 Hz

Camera ──── C1 ─────── C2 ─────── C3 ─────── C4 ───────
            30 Hz

SyncEngine takes each LiDAR frame as reference and finds the
nearest IMU sample and Camera frame within ±tolerance_ns.

Bundle 1:  L1 + I_nearest + C_nearest
Bundle 2:  L2 + I_nearest + C_nearest
...
```

**Algorithm** (per sync cycle):
1. Pop the oldest unconsumed LiDAR frame → `ref_ts`
2. Search IMU queue for sample closest to `ref_ts` within tolerance
3. Search Camera queue for frame closest to `ref_ts` within tolerance
4. If Camera is available, emit `SyncBundle`; evict consumed data
5. If not, wait until next poll cycle

---

## CI/CD & Security

Four GitHub Actions workflows implement a three-tier artifact promotion model
(dev → staging → production):

| Workflow | Trigger | Purpose |
|----------|---------|---------|
| `ci.yml` | Push/PR | Build matrix + smoke tests + dev Docker publish |
| `release.yml` | `vX.Y.Z` / `vX.Y.Z-rc.N` tag | Full release (`.deb`, `.whl`, Docker, GitHub Release) |
| `promote.yml` | Manual | Zero-rebuild RC → production promotion |
| `security.yml` | Push/PR + weekly | CodeQL, Trivy, SBOM, Scorecard, secret scan |

**Security**: Cosign keyless signing (Sigstore OIDC), SLSA L2+ provenance,
Syft SBOM, Trivy container scanning, Dependabot, OpenSSF Scorecard.
See [SECURITY.md](SECURITY.md) for vulnerability reporting.
See [ENVIRONMENTS.md](ENVIRONMENTS.md) for environment protection rules.

> **Full CI/CD architecture**, caching strategy, release pipeline stages,
> artifact promotion model, and security details are documented in
> [`.github/copilot-instructions.md`](.github/copilot-instructions.md).

### Release Artifacts

A single tag push (`git tag v1.2.3 && git push --tags`) produces:

| Artifact | Format | Platforms |
|----------|--------|-----------|
| Debian runtime pkg | `libthunderbird-sdk0_*.deb` | amd64, arm64 |
| Debian dev pkg | `libthunderbird-sdk-dev_*.deb` | amd64, arm64 |
| Python wheels | `.whl` (manylinux2014) | x86_64, aarch64 × CPython 3.9–3.12 |
| Docker runtime image | OCI multi-arch | linux/amd64, linux/arm64 |
| Docker dev image | OCI multi-arch | linux/amd64, linux/arm64 |
| Checksums | `SHA256SUMS.txt` + cosign signature | — |

### Creating a Release

```bash
# 1. Update CMakeLists.txt: project(thunderbird_sdk VERSION X.Y.Z ...)
# 2. Commit and tag:
git commit -am "release: v1.2.3"
git tag v1.2.3
git push origin main --tags
# 3. CI builds all artifacts with version injected automatically
```

For the promoted-release workflow (RC → production), see [ENVIRONMENTS.md](ENVIRONMENTS.md).

---

### Docker Images

Two multi-arch images (linux/amd64 + linux/arm64) are published to GHCR on every release:

| Image | Base | Contents | Size |
|-------|------|----------|------|
| `thunderbird-runtime` | Ubuntu 24.04 | Shared library only (`.so.*`) | ~30 MB |
| `thunderbird-dev` | Ubuntu 24.04 | Headers, static + shared libs, pkg-config, CMake config, build tools, Python, source tree, tests | ~400 MB |

#### Tag strategy

Each release produces four tags per image:

```
ghcr.io/OWNER/thunderbird-runtime:1.2.3    # exact version
ghcr.io/OWNER/thunderbird-runtime:1.2      # minor track
ghcr.io/OWNER/thunderbird-runtime:1        # major track
ghcr.io/OWNER/thunderbird-runtime:latest   # latest stable
```

#### Pull & run

```bash
# Pull the minimal runtime image (shared lib only, non-root)
docker pull ghcr.io/OWNER/thunderbird-runtime:1.2.3

# Pull the full dev environment
docker pull ghcr.io/OWNER/thunderbird-dev:1.2.3

# Run the dev container interactively
docker run --rm -it ghcr.io/OWNER/thunderbird-dev:1.2.3
# Inside: gcc, cmake, python3, gdb, valgrind all available
# SDK pre-installed: pkg-config --cflags --libs thunderbird-sdk
```

#### Build locally

```bash
# Single-arch (native)
docker build --build-arg VERSION=0.1.0 \
  -f docker/Dockerfile.runtime -t thunderbird-runtime .

# Multi-arch (requires buildx)
docker buildx create --use
docker buildx build --platform linux/amd64,linux/arm64 \
  --build-arg VERSION=0.1.0 \
  -f docker/Dockerfile.runtime -t thunderbird-runtime .
```

#### Verify image signature

```bash
cosign verify \
  --certificate-identity-regexp "github.com/OWNER/thunderbird" \
  --certificate-oidc-issuer "https://token.actions.githubusercontent.com" \
  ghcr.io/OWNER/thunderbird-runtime:1.2.3
```

### Debian Packages

The SDK produces two Debian packages following Debian policy 4.6.2:

| Package | Contents | Install path |
|---------|----------|-------------|
| `libthunderbird-sdk0` | Shared library (`libthunderbird_sdk.so.0.*`) | `/usr/lib/<multiarch>/` |
| `libthunderbird-sdk-dev` | Headers, static lib (`.a`), dev symlink (`.so`), pkg-config, CMake config | `/usr/include/thunderbird/`, `/usr/lib/<multiarch>/` |

**Build locally:**

```bash
# Install build deps
sudo apt-get install build-essential cmake pkg-config debhelper devscripts

# Stamp the changelog (version from CMakeLists.txt)
./scripts/update-changelog.sh

# Build both .deb packages
dpkg-buildpackage -us -uc -b

# Packages appear in parent directory
ls ../*.deb
# libthunderbird-sdk0_0.1.0_amd64.deb
# libthunderbird-sdk-dev_0.1.0_amd64.deb
```

**Install from GitHub Release:**

```bash
# Download the latest release .deb packages (replace VERSION and ARCH)
# ARCH is typically amd64 or arm64
VERSION="0.1.0"
ARCH="amd64"
gh release download "v${VERSION}" \
  --repo huajwang/thunderbird \
  --pattern "libthunderbird-sdk0_${VERSION}_${ARCH}.deb" \
  --pattern "libthunderbird-sdk-dev_${VERSION}_${ARCH}.deb"

# Install runtime + dev
sudo dpkg -i libthunderbird-sdk0_*.deb libthunderbird-sdk-dev_*.deb
```

Or download manually from the [Releases](https://github.com/huajwang/thunderbird/releases) page and then:

```bash
# Install the downloaded .deb files
sudo dpkg -i libthunderbird-sdk0_*.deb libthunderbird-sdk-dev_*.deb
```

**Install from local build:**

```bash
# After running dpkg-buildpackage, packages are in the parent directory
sudo dpkg -i ../libthunderbird-sdk0_*.deb ../libthunderbird-sdk-dev_*.deb
```

**Use the installed SDK:**

```bash
# Use via pkg-config
g++ my_app.cpp $(pkg-config --cflags --libs thunderbird-sdk) -o my_app

# Use via CMake
#   find_package(ThunderbirdSDK REQUIRED)
#   target_link_libraries(my_app PRIVATE Thunderbird::thunderbird_sdk_shared)
```

### Python Wheels

The SDK publishes PEP 599-compliant **manylinux2014** wheels for CPython 3.9–3.12
on both x86_64 and aarch64:

| Wheel tag | Platform | Python | glibc |
|-----------|----------|--------|-------|
| `manylinux2014_x86_64` | x86_64 | cp39–cp312 | ≥ 2.17 |
| `manylinux2014_aarch64` | aarch64 | cp39–cp312 | ≥ 2.17 |

**Build pipeline:** scikit-build-core compiles the C++ SDK + pybind11 extension
inside the official PyPA manylinux2014 container (CentOS 7, devtoolset-12), then
`auditwheel repair` bundles any non-whitelisted shared libraries and applies the
correct platform tag. `libstdc++` is statically linked (`-static-libstdc++`) so
wheels work on any system with glibc ≥ 2.17.

```bash
# Install from release wheel (once published to PyPI or GH Release)
pip install spatial-sdk

# Build from source (requires C++ compiler + CMake ≥ 3.16)
pip install .

# Build wheels locally with cibuildwheel
pip install cibuildwheel
cibuildwheel --output-dir dist/

# Build a single wheel (useful for development)
pip wheel . --no-deps -w dist/
```

---

## Versioning Strategy

**Single source of truth: Git tag** (`vX.Y.Z`). The version propagates automatically
to C++ headers, Python wheels, Debian packages, and Docker images via CI.

### Checking the version at compile time

```cpp
#include <thunderbird/version.h>

static_assert(THUNDERBIRD_VERSION_AT_LEAST(1, 0, 0), "Need SDK ≥ 1.0.0");
printf("SDK %s\n", thunderbird::version());  // "1.2.3"
```

### Checking the version in Python

```python
import spatial_sdk
print(spatial_sdk.__version__)  # "1.2.3"
```

> **Version propagation internals** (extract-version.sh, CMake override,
> Python resolution chain, Debian substitution, Docker build-arg) are
> documented in [`.github/copilot-instructions.md`](.github/copilot-instructions.md).

---

## Implementation Plan (step-by-step)

### Phase 1 — Core infrastructure (this PoC) ✅
1. Define data types with dual timestamps (hardware + host)
2. Implement `ITransport` and `SimulatedTransport`
3. Build per-sensor driver interface + simulated backends
4. Implement lock-free `RingBuffer` for high-throughput paths
5. Build `SyncEngine` with nearest-neighbour alignment
6. Wire everything into `DeviceManager` with PImpl pattern
7. Add C++ examples and unit tests
8. Create ROS 2 bridge node (PointCloud2, Imu, Image publishers)
9. Add Python bindings via pybind11

### Phase 2 — Hardware integration (next)
- [ ] Implement real USB/Ethernet transport drivers
- [ ] Parse vendor-specific packet formats (LiDAR, IMU, Camera)
- [ ] Hardware PTP/PPS timestamp recovery
- [ ] Clock-offset estimation (linear regression on HW↔host timestamps)
- [ ] Reconnection / watchdog logic

### Phase 3 — Production hardening
- [ ] Configurable logging framework (spdlog)
- [ ] Thread-safe statistics / diagnostics endpoint
- [ ] CI pipeline (CMake presets, cross-compilation, sanitizers)
- [ ] API versioning & ABI stability guarantees
- [ ] Documentation generation (Doxygen)

### Phase 4 — Perception-ready extensions
- [ ] GPU-accelerated point cloud preprocessing
- [ ] Camera intrinsic / extrinsic calibration storage
- [ ] Multi-device support (multiple `DeviceManager` instances)
- [ ] Recording / playback (rosbag2 or custom binary format)

---

## License

MIT — see `LICENSE` for details.
