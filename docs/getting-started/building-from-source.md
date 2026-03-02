# Building from Source

Detailed build instructions for all supported platforms and configurations.

---

## Build Matrix

| Platform | Compiler | Architecture | Status |
|----------|----------|-------------|--------|
| Ubuntu 22.04 / 24.04 | GCC 12+ | x86_64, aarch64 | ✅ Primary |
| Debian 12 | GCC 12+ | x86_64, aarch64 | ✅ Supported |
| macOS 13+ | Apple Clang 15+ | x86_64, arm64 | ✅ Supported |
| Windows (Cygwin/MSYS2) | GCC 10+ | x86_64 | ✅ Supported |
| Windows (MSVC) | MSVC 2019+ | x86_64 | 🔲 Planned |

---

## Standard Build

```bash
git clone https://github.com/huajwang/thunderbird.git
cd thunderbird

cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel $(nproc)

# Verify
cd build && ctest --output-on-failure
```

---

## CMake Options

| Option | Default | Description |
|--------|---------|-------------|
| `CMAKE_BUILD_TYPE` | `Release` | `Debug`, `Release`, `RelWithDebInfo`, `MinSizeRel` |
| `THUNDERBIRD_USE_SIMULATED` | `ON` | Build with simulated sensor backends |
| `THUNDERBIRD_BUILD_PYTHON` | `OFF` | Build Python bindings (requires pybind11) |
| `THUNDERBIRD_BUILD_EXAMPLES` | `ON` | Build example programs |
| `THUNDERBIRD_BUILD_TESTS` | `ON` | Build unit tests |
| `THUNDERBIRD_ENABLE_GPU_PERCEPTION` | `OFF` | Build GPU detector backends (requires CUDA + TensorRT) |
| `THUNDERBIRD_LOG_LEVEL_COMPILE` | — | Strip log calls below this level (0=trace .. 6=off) |
| `THUNDERBIRD_VERSION_OVERRIDE` | — | Override version string (usually set by CI) |

```bash
# Example: Release build with Python, no tests, log level = warn
cmake -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DTHUNDERBIRD_BUILD_PYTHON=ON \
    -DTHUNDERBIRD_BUILD_TESTS=OFF \
    -DTHUNDERBIRD_LOG_LEVEL_COMPILE=3
```

---

## Cross-Compilation (aarch64)

```bash
cmake -B build-arm64 \
    -DCMAKE_TOOLCHAIN_FILE=/path/to/aarch64-linux-gnu.cmake \
    -DCMAKE_BUILD_TYPE=Release

cmake --build build-arm64 --parallel
```

---

## Docker Build

```bash
# Development image (includes build tools + SDK)
docker build -f docker/Dockerfile.dev -t thunderbird-dev .

# Runtime image (shared library only)
docker build -f docker/Dockerfile.runtime -t thunderbird-runtime .

# Build inside Docker
docker run --rm -v $(pwd):/workspace -w /workspace thunderbird-dev \
    bash -c "cmake -B build && cmake --build build --parallel && cd build && ctest"
```

---

## Debian Packages

```bash
# Build .deb packages
./packaging/build_deb.sh

# Install
sudo dpkg -i libthunderbird-sdk0_*.deb libthunderbird-sdk-dev_*.deb
```

Packages produced:
- `libthunderbird-sdk0` — shared library (`libthunderbird_sdk.so.*`)
- `libthunderbird-sdk-dev` — headers, static library, CMake config, pkg-config

---

## Python Wheels

```bash
pip install scikit-build-core pybind11
pip install .

# Or build wheel directly
pip wheel . -w dist/
```

Import as:
```python
import spatial_sdk as tb
```

---

## GPU Perception (Optional)

Requires CUDA ≥ 11.4 and TensorRT ≥ 8.x.

```bash
cmake -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DTHUNDERBIRD_ENABLE_GPU_PERCEPTION=ON \
    -DTENSORRT_ROOT=/usr/local/TensorRT

cmake --build build --parallel
```

---

## Generating API Documentation

```bash
# Install dependencies
sudo apt-get install doxygen graphviz   # Linux
brew install doxygen graphviz           # macOS

# Generate
cmake --build build --target docs

# Or directly
THUNDERBIRD_VERSION=0.2.0 doxygen docs/api/Doxyfile

# View
open build/docs/html/index.html
```
