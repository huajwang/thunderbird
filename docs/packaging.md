# Packaging — Docker, Debian & Python Wheels

## Docker Images

Two multi-arch images (linux/amd64 + linux/arm64) are published to GHCR on every release:

| Image | Base | Contents | Size |
|-------|------|----------|------|
| `thunderbird-runtime` | Ubuntu 24.04 | Shared library only (`.so.*`) | ~30 MB |
| `thunderbird-dev` | Ubuntu 24.04 | Headers, static + shared libs, pkg-config, CMake config, build tools, Python, source tree, tests | ~400 MB |

### Tag strategy

Each release produces four tags per image:

```
ghcr.io/OWNER/thunderbird-runtime:1.2.3    # exact version
ghcr.io/OWNER/thunderbird-runtime:1.2      # minor track
ghcr.io/OWNER/thunderbird-runtime:1        # major track
ghcr.io/OWNER/thunderbird-runtime:latest   # latest stable
```

### Pull & run

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

### Build locally

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

### Verify image signature

```bash
cosign verify \
  --certificate-identity-regexp "github.com/OWNER/thunderbird" \
  --certificate-oidc-issuer "https://token.actions.githubusercontent.com" \
  ghcr.io/OWNER/thunderbird-runtime:1.2.3
```

---

## Debian Packages

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

---

## Python Wheels

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
