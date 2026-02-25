# Thunderbird SDK — Copilot Instructions

This file provides AI coding agents (GitHub Copilot, etc.) with the project
context needed to make accurate, consistent contributions.

---

## Project Overview

Thunderbird is a **C++20 multi-sensor device SDK** (LiDAR + IMU + Camera) with
Python bindings (pybind11), ROS 2 bridge, Debian packaging, and Docker images.
Current phase: **PoC** — simulated sensor backends only, no real hardware yet.

**Version**: defined in `CMakeLists.txt` → `project(thunderbird_sdk VERSION X.Y.Z)`.

---

## Repository Layout

```
thunderbird/
├── CMakeLists.txt                 # Top-level build (project version source-of-truth)
├── pyproject.toml                 # Python packaging (scikit-build-core + cibuildwheel)
├── .github/
│   ├── copilot-instructions.md    # THIS FILE — AI agent context
│   ├── dependabot.yml             # Automated dependency updates
│   └── workflows/
│       ├── ci.yml                 # CI on push/PR → build + test + dev publish
│       ├── promote.yml            # RC → production promotion (zero-rebuild)
│       ├── release.yml            # Tag-triggered release pipeline (vX.Y.Z + RC)
│       └── security.yml           # CodeQL, Trivy, SBOM, Scorecard, secret scan
├── ENVIRONMENTS.md                # Environment protection rules (dev/staging/prod)
├── SECURITY.md                    # Vulnerability reporting policy
├── debian/                        # Debian packaging (Debian policy 4.6.2)
├── docker/
│   ├── Dockerfile.runtime         # Multi-stage → minimal runtime (shared lib only)
│   └── Dockerfile.dev             # Multi-stage → full dev (tools + SDK + source)
├── packaging/
│   └── build_deb.sh               # Quick .deb builder (legacy)
├── scripts/
│   ├── extract-version.sh         # Version extraction utility
│   ├── generate-release-notes.sh  # Auto-changelog from conventional commits
│   └── update-changelog.sh        # Stamp debian/changelog from version
├── sdk/
│   ├── CMakeLists.txt             # Builds static + shared lib, install rules
│   ├── include/thunderbird/       # Public API headers
│   └── src/                       # Implementation
├── examples/                      # C++ example programs
├── tests/                         # Unit tests (CTest)
├── python/                        # pybind11 bindings + example
└── ros2_bridge/                   # ROS 2 node
```

---

## Build System

- **CMake ≥ 3.16**, C++20 (`CMAKE_CXX_STANDARD 20`)
- Key CMake options:
  - `THUNDERBIRD_BUILD_TESTS` (ON/OFF)
  - `THUNDERBIRD_BUILD_EXAMPLES` (ON/OFF)
  - `THUNDERBIRD_BUILD_PYTHON` (OFF by default)
  - `THUNDERBIRD_BUILD_ROS2` (OFF by default)
  - `THUNDERBIRD_USE_SIMULATED` (ON — always ON until real hardware exists)
  - `THUNDERBIRD_VERSION_OVERRIDE` (X.Y.Z — CI injects from Git tag)
- **Version override**: must be strict `X.Y.Z` format — no suffixes like `-dev`
  or `-rc.N` (CMakeLists.txt validates with regex and `FATAL_ERROR`).

### Build commands

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
cd build && ctest --output-on-failure
```

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        User Application                             │
│   on_lidar()  on_imu()  on_camera()  on_sync()                     │
└──────────┬──────────┬──────────┬──────────┬────────────────────────┘
           │          │          │          │
┌──────────▼──────────▼──────────▼──────────▼────────────────────────┐
│                      DeviceManager (public API, PImpl)              │
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

### Key design patterns

| Pattern | Where | Rationale |
|---------|-------|-----------|
| `shared_ptr<const T>` for frames | All callbacks | Zero-copy fan-out; immutable after creation |
| One thread per sensor driver | `SensorDriver` | Matches hardware I/O rates; avoids blocking |
| Lock-free SPSC `RingBuffer` | High-throughput paths | Drops oldest on overflow; no mutex |
| Nearest-neighbour sync | `SyncEngine` | Simple, deterministic, configurable tolerance |
| PImpl in `DeviceManager` | Public API | Stable ABI; hides internals |

---

## CI/CD Pipeline

### Workflows

| File | Trigger | Purpose |
|------|---------|---------|
| `ci.yml` | Push/PR to main/develop | Build matrix (Release + Debug + ARM64), deb/wheel/Docker smoke tests, dev Docker publish |
| `release.yml` | `vX.Y.Z` or `vX.Y.Z-rc.N` tag | Full release: .deb, .whl, Docker, GitHub Release |
| `promote.yml` | Manual dispatch | Zero-rebuild promotion from RC to production (crane copy) |
| `security.yml` | Push/PR + weekly schedule | CodeQL, Dependency Review, Trivy, SBOM, TruffleHog, Scorecard |

### Artifact promotion model

```
Dev  (:dev-<sha7>)           ← CI push to main  (ci.yml)
Staging (:X.Y.Z-rc.N)       ← RC tag push       (release.yml)
Production (:X.Y.Z :X.Y :X) ← vX.Y.Z tag OR promote.yml
```

- **Environments**: `dev`, `staging`, `production` (see ENVIRONMENTS.md)
- **production** requires reviewer approval
- Docker production tags are **write-once** (immutable artifact policy)

### Caching strategy

| Layer | Tool | Key |
|-------|------|-----|
| Compiler output | ccache | Hash of `CMakeLists.txt`, `sdk/**`, `tests/**`, `examples/**` |
| Python packages | pip cache | `actions/setup-python` built-in |
| Apt packages | `actions/cache` | `~/apt-cache` per runner |
| Docker layers | BuildKit GHA | Scoped per image variant |

### CI job names (for branch protection status checks)

- `linux-x64 / Release`
- `linux-x64-debug / Debug`
- `linux-arm64 / Release`
- `Deb smoke`
- `Wheel smoke`
- `Docker runtime` / `Docker dev`
- `Dev publish (runtime)` / `Dev publish (dev)`

---

## Version Flow

```
Git tag (vX.Y.Z)
    │
    ├─► scripts/extract-version.sh ─► "X.Y.Z" string
    │       │
    │       ├─► CMake: -DTHUNDERBIRD_VERSION_OVERRIDE=X.Y.Z
    │       │       ├─► version.h.in → version.h (compile-time macros)
    │       │       └─► thunderbird::version() (runtime)
    │       │
    │       ├─► cibuildwheel: CIBW_CONFIG_SETTINGS cmake.args=-D…
    │       │       └─► spatial_sdk wheel X.Y.Z
    │       │
    │       ├─► Docker build-arg VERSION=X.Y.Z
    │       │       └─► OCI label + CMake override inside container
    │       │
    │       └─► debian/changelog stamp
    │
    └─► GitHub Release tag
```

### extract-version.sh modes

```bash
./scripts/extract-version.sh                      # auto: tag → CMakeLists.txt
./scripts/extract-version.sh --from-tag            # Git tag only
./scripts/extract-version.sh --from-cmake          # CMakeLists.txt only
./scripts/extract-version.sh --component major     # → "1"
./scripts/extract-version.sh --check-consistency   # verify tag == CMakeLists.txt
```

### CMake version embedding

```cmake
# CI: cmake -DTHUNDERBIRD_VERSION_OVERRIDE=1.2.3 ..
# Local dev: uses project(VERSION ...) as fallback
if(DEFINED THUNDERBIRD_VERSION_OVERRIDE)
    # validates X.Y.Z regex, then overrides PROJECT_VERSION
endif()
```

`version.h` is generated via `configure_file()` providing:
- `THUNDERBIRD_VERSION_STRING` — e.g. `"1.2.3"`
- `THUNDERBIRD_VERSION_MAJOR/MINOR/PATCH`
- `THUNDERBIRD_VERSION_AT_LEAST(major, minor, patch)` macro

### Python version resolution (priority chain)

1. `importlib.metadata` (set by scikit-build-core for installed wheels)
2. C++ extension `_spatial_sdk_core.THUNDERBIRD_VERSION` (from version.h)
3. Fallback `"0.0.0.dev0"` for editable installs

`pyproject.toml` reads fallback version from CMakeLists.txt via regex.

---

## Packaging Details

### Debian (.deb)

- **Two packages**: `libthunderbird-sdk0` (runtime .so) + `libthunderbird-sdk-dev` (headers, .a, .pc, cmake config)
- Built via `dpkg-buildpackage` in Ubuntu 22.04 container
- `debian/rules` uses debhelper 13 with CMake overrides
- `scripts/update-changelog.sh` stamps `debian/changelog`

### Python wheels

- **manylinux2014** (PEP 599, glibc ≥ 2.17)
- CPython 3.9–3.12, x86_64 + aarch64
- scikit-build-core + pybind11
- `auditwheel repair` bundles non-whitelisted .so; `-static-libstdc++`
- Config in `pyproject.toml` → `[tool.cibuildwheel]`

### Docker images

- `thunderbird-runtime`: Ubuntu 24.04 minimal + shared lib (~30 MB)
- `thunderbird-dev`: Ubuntu 24.04 + full toolchain + SDK + source (~400 MB)
- Multi-arch: linux/amd64 + linux/arm64 via buildx + QEMU
- Tag hierarchy: `:X.Y.Z`, `:X.Y`, `:X`, `:latest`
- Two-stage Dockerfile: builder → DESTDIR install → COPY to final stage

---

## Security

- **CodeQL**: C++ & Python static analysis on every push/PR + weekly
- **Dependency Review**: blocks high/critical CVEs + copyleft licenses on PRs
- **Trivy**: container image CVE scanning (CRITICAL/HIGH block release)
- **TruffleHog**: secret scanning across full commit history
- **OpenSSF Scorecard**: supply chain health assessment
- **Cosign**: keyless signing (Sigstore OIDC) for Docker images + checksums
- **SLSA L2+**: `actions/attest-build-provenance` for .deb + .whl
- **SBOM**: Syft SPDX + CycloneDX for source and containers
- **Workflow permissions**: least-privilege (`contents: read` default, per-job escalation)
- **No long-lived secrets**: everything uses Sigstore OIDC keyless

See `SECURITY.md` for vulnerability reporting policy.

---

## Coding Conventions

- **C++20** standard, `-Wall -Wextra -Wpedantic`
- **Namespaces**: all SDK code in `thunderbird::`
- **Smart pointers**: `shared_ptr<const T>` for data frames (immutable after creation)
- **Headers**: `#pragma once`, include what you use
- **Tests**: CTest-based, one `test_*.cpp` per component
- **Commits**: conventional commits (`feat:`, `fix:`, `perf:`, `docs:`, `ci:`, etc.)
- **Suppress unused-variable warnings**: `(void)var;` cast pattern
- **File naming**: `snake_case.cpp`, `snake_case.h`

---

## Common Tasks

### Adding a new sensor driver

1. Create `sdk/include/thunderbird/drivers/simulated_<sensor>.h`
2. Implement `ISensorDriver` interface
3. Wire into `DeviceManager` constructor
4. Add test in `tests/test_<sensor>.cpp`
5. Register in `tests/CMakeLists.txt`

### Adding a new CI job

1. Add job definition in the appropriate workflow file
2. If independent, do NOT add `needs:` — let it run in parallel
3. Use ccache + apt cache pattern from existing build jobs
4. Add job name to branch protection status checks if it should gate merges

### Cutting a release

1. Update `CMakeLists.txt`: `project(thunderbird_sdk VERSION X.Y.Z ...)`
2. Commit: `git commit -am "release: vX.Y.Z"`
3. Tag RC: `git tag v1.2.3-rc.1 && git push origin v1.2.3-rc.1`
4. Validate in staging, then promote via Actions → Promote workflow
5. Alternatively: `git tag v1.2.3 && git push origin v1.2.3` for direct release

---

## Implementation Phases

### Phase 1 — Core infrastructure (PoC) ✅
- Data types, ITransport, SimulatedTransport
- Per-sensor drivers (simulated LiDAR, IMU, Camera)
- Lock-free RingBuffer, SyncEngine, DeviceManager (PImpl)
- C++ examples, unit tests, ROS 2 bridge, Python bindings

### Phase 2 — Hardware integration (next)
- Real USB/Ethernet transport drivers
- Vendor-specific packet parsing
- Hardware PTP/PPS timestamp recovery
- Clock-offset estimation, reconnection/watchdog

### Phase 3 — Production hardening
- Configurable logging (spdlog)
- Thread-safe diagnostics
- API versioning & ABI stability
- Doxygen documentation

### Phase 4 — Perception-ready extensions
- GPU point cloud preprocessing
- Camera calibration storage
- Multi-device support
- Recording / playback
