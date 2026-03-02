# Thunderbird SDK — Documentation Architecture

> Canonical reference for the documentation structure, conventions, and build
> pipeline for the Thunderbird SDK.

---

## Folder Structure

```
thunderbird/
├── README.md                         # Project landing page (see outline below)
├── CHANGELOG.md                      # Auto-generated from Git tags + PR labels
├── SECURITY.md                       # Vulnerability reporting policy (exists)
├── ENVIRONMENTS.md                   # CI/CD environment rules (exists)
│
├── docs/
│   ├── DOCUMENTATION_ARCHITECTURE.md # ← this file
│   │
│   ├── getting-started/
│   │   ├── quick-start.md            # 5-minute zero-hardware walkthrough
│   │   ├── hardware-setup.md         # Supported devices, wiring, firmware
│   │   └── building-from-source.md   # Full build matrix, cross-compile, Docker
│   │
│   ├── guides/
│   │   ├── time-synchronization.md   # Clock domains, drift, SyncedFrame
│   │   ├── advanced-configuration.md # YAML profiles, env vars, all CMake flags
│   │   ├── perception-pipeline.md    # Detector + tracker tuning
│   │   ├── recording-playback.md     # .tbrec format, offline analysis
│   │   └── diagnostics.md           # DiagnosticsManager, JSON metrics
│   │
│   ├── api/
│   │   ├── overview.md               # Header tiering, namespace layout
│   │   ├── Doxyfile                  # Doxygen configuration
│   │   ├── doxygen-awesome.css       # (optional) Doxygen Awesome theme
│   │   └── mainpage.dox             # Doxygen @mainpage content
│   │
│   ├── reference/
│   │   ├── cmake-options.md          # Table of every CMake option
│   │   ├── environment-variables.md  # Runtime env vars
│   │   └── error-codes.md           # Status enum reference
│   │
│   ├── troubleshooting/
│   │   └── troubleshooting.md        # Symptom → cause → fix tables
│   │
│   ├── release/
│   │   ├── release-notes-template.md # Template for each release
│   │   └── v0.2.0.md               # Current release notes
│   │
│   ├── contributing/
│   │   └── code-snippet-style.md     # Example formatting conventions
│   │
│   └── design/                       # (existing design docs, relocated)
│       ├── CLOCK_SYNC_DESIGN.md
│       ├── DEVICE_HEALTH_MONITOR_DESIGN.md
│       ├── LIDAR_FRAME_ASSEMBLER_DESIGN.md
│       ├── PERCEPTION_LAYER_DESIGN.md
│       ├── PHASE2_TRANSPORT_DESIGN.md
│       └── VENDOR_PARSER_DESIGN.md
│
├── examples/                         # Runnable code (existing)
│   ├── basic_streaming.cpp
│   ├── sync_demo.cpp
│   └── ...
│
└── sdk/include/thunderbird/          # Public headers (Doxygen source)
```

### Rationale

| Principle | Implementation |
|-----------|----------------|
| **Progressive disclosure** | `getting-started/` → `guides/` → `api/` → `reference/` |
| **Searchability** | Flat names, no nesting beyond 2 levels |
| **Separation of concerns** | Design docs (internal) vs. guides (user-facing) vs. API ref (generated) |
| **CI-friendly** | `docs/api/Doxyfile` can be invoked by `doxygen docs/api/Doxyfile` in CI |

---

## Doxygen Configuration Strategy

### Goals

1. **Generate from existing `///` comments** — no comment-style migration needed.
2. **Tiered header visibility** — only Tier 1 (public API) and Tier 2 (extension) headers documented;
   internal headers excluded.
3. **Example integration** — code samples from `examples/` embedded via `@example`.
4. **Offline + CI** — runs headless in CI, outputs `build/docs/html/`.
5. **Theme** — Doxygen Awesome CSS for modern look (optional, zero build deps).

### Key Doxyfile Settings

```ini
# ── Project ──────────────────────────────────────────────────────
PROJECT_NAME           = "Thunderbird SDK"
PROJECT_NUMBER         = $(THUNDERBIRD_VERSION)     # injected by CI/CMake
PROJECT_BRIEF          = "Multi-sensor robotics SDK for LiDAR + IMU + Camera"

# ── Input ────────────────────────────────────────────────────────
INPUT                  = sdk/include/thunderbird \
                         docs/api/mainpage.dox \
                         examples
FILE_PATTERNS          = *.h *.dox *.cpp
EXCLUDE_PATTERNS       = */detail/* */drivers/* */decoders/*
EXCLUDE                = sdk/include/thunderbird/simulated_transport.h \
                         sdk/include/thunderbird/protocol.h
RECURSIVE              = YES
EXAMPLE_PATH           = examples
EXAMPLE_PATTERNS       = *.cpp *.py

# ── Parsing ──────────────────────────────────────────────────────
EXTRACT_ALL            = NO         # Only document commented entities
EXTRACT_PRIVATE        = NO
EXTRACT_STATIC         = NO
HIDE_UNDOC_MEMBERS     = YES
HIDE_UNDOC_CLASSES     = YES
JAVADOC_AUTOBRIEF      = YES        # First sentence = @brief
QT_AUTOBRIEF           = YES

# ── Output ───────────────────────────────────────────────────────
OUTPUT_DIRECTORY       = build/docs
GENERATE_HTML          = YES
GENERATE_LATEX         = NO
HTML_OUTPUT            = html
HTML_EXTRA_STYLESHEET  = docs/api/doxygen-awesome.css

# ── Diagrams ─────────────────────────────────────────────────────
HAVE_DOT               = YES        # Class/collaboration/call graphs
DOT_IMAGE_FORMAT       = svg
CLASS_GRAPH            = YES
COLLABORATION_GRAPH    = YES
INCLUDE_GRAPH          = YES
CALL_GRAPH             = NO         # too noisy
CALLER_GRAPH           = NO

# ── Namespaces ───────────────────────────────────────────────────
HIDE_SCOPE_NAMES       = NO
SHOW_NAMESPACES        = YES

# ── Source browser ───────────────────────────────────────────────
SOURCE_BROWSER         = YES
INLINE_SOURCES         = NO
STRIP_CODE_COMMENTS    = YES

# ── Warnings ─────────────────────────────────────────────────────
WARN_IF_UNDOCUMENTED   = YES
WARN_IF_DOC_ERROR      = YES
WARN_AS_ERROR          = FAIL_ON_WARNINGS   # CI gate
```

### CMake Integration

```cmake
find_package(Doxygen OPTIONAL_COMPONENTS dot)
if(DOXYGEN_FOUND)
    set(DOXYGEN_IN  ${CMAKE_SOURCE_DIR}/docs/api/Doxyfile)
    set(DOXYGEN_OUT ${CMAKE_BINARY_DIR}/docs)

    add_custom_target(docs
        COMMAND ${CMAKE_COMMAND} -E env
            THUNDERBIRD_VERSION=${PROJECT_VERSION}
            ${DOXYGEN_EXECUTABLE} ${DOXYGEN_IN}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM
    )
endif()
```

Usage: `cmake --build build --target docs`

### CI Pipeline Stage

```yaml
- name: Generate docs
  run: |
    sudo apt-get install -y doxygen graphviz
    THUNDERBIRD_VERSION=${{ github.ref_name }} doxygen docs/api/Doxyfile

- name: Deploy to GitHub Pages
  if: startsWith(github.ref, 'refs/tags/v')
  uses: peaceiris/actions-gh-pages@v3
  with:
    publish_dir: build/docs/html
```

---

## README.md Outline

The root README should be a **concise landing page** (max ~200 lines) that links to
deeper content in `docs/`.  Current README is 490 lines and mixes user guide,
contributor guide, and release engineering — these should be split.

### Proposed Outline

```markdown
# Thunderbird SDK

One-line description.  Badges (CI, version, license, docs).

## What is Thunderbird?
- 2–3 sentence elevator pitch
- Supported sensors (LiDAR, IMU, Camera)
- Key capabilities (time sync, SLAM, 3D perception, recording)

## Architecture
- Simplified ASCII diagram (existing)
- Link to → docs/guides/ for details

## Quick Start
- Prerequisites (3 bullets)
- Build command block (4 lines)
- Run first example (1 line)
- Link to → docs/getting-started/quick-start.md

## Documentation
- Table of links:
  | Guide                      | Description                        |
  |----------------------------|------------------------------------|
  | Quick Start                | Zero-hardware walkthrough          |
  | Hardware Setup             | Wiring, firmware, supported boards |
  | Time Synchronization       | Clock domains and SyncedFrame API  |
  | API Reference (Doxygen)    | Full class/function documentation  |
  | Advanced Configuration     | YAML profiles, CMake flags         |
  | Perception Pipeline        | 3D object detection & tracking     |
  | Recording & Playback       | .tbrec format, offline analysis    |
  | Diagnostics                | Runtime health & perf metrics      |
  | Troubleshooting            | Common issues & fixes              |

## Language Bindings
- C++ (primary), Python (`spatial-sdk`), ROS 2 bridge
- Links to → examples/python_example.py, ros2_bridge/

## Installation
- Debian packages (2 lines)
- Python wheel (1 line)
- Docker images (2 lines)
- Link to → docs/getting-started/building-from-source.md

## Release Notes
- Link to → docs/release/v0.2.0.md
- Link to → CHANGELOG.md

## Contributing
- Link to → CONTRIBUTING.md (future)
- Link to → docs/contributing/code-snippet-style.md

## License
MIT
```

---

## Code Snippet Format

All documentation examples must follow these conventions.  See
[docs/contributing/code-snippet-style.md](contributing/code-snippet-style.md) for the
full reference.

### Rules

1. **Compilable** — every C++ snippet must compile with the SDK headers.
2. **Self-contained** — include all necessary `#include` lines.
3. **Minimal** — demonstrate one concept per snippet.
4. **Error-handling shown** — check `Status` return values.
5. **Comments explain "why"** — not "what".

### C++ Template

````cpp
/// @file example_name.cpp
/// @brief One-sentence description of what this demonstrates.
///
/// Requires: Thunderbird SDK >= 0.2.0

#include <thunderbird/device_manager.h>
#include <cstdio>

int main() {
    // 1. Configure the device (simulated mode — no hardware).
    thunderbird::DeviceConfig config;
    config.lidar_hz = 10.0;

    // 2. Create and connect.
    thunderbird::DeviceManager device(config);
    if (device.connect() != thunderbird::Status::OK) {
        std::fprintf(stderr, "Connection failed\n");
        return 1;
    }

    // 3. Register a callback.
    device.on_lidar([](auto frame) {
        std::printf("LiDAR frame: %zu points\n", frame->points.size());
    });

    // 4. Stream for a short duration.
    device.start();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    device.stop();

    device.disconnect();
    return 0;
}
````

### Python Template

```python
"""one_sentence_description.py — What this demonstrates.

Requires: spatial-sdk >= 0.2.0
"""
import spatial_sdk as tb

# 1. Connect (simulated).
device = tb.DeviceManager()
device.connect()

# 2. Stream and collect frames.
device.start()
frames = device.pull_lidar(timeout_ms=2000, max_frames=5)
print(f"Got {len(frames)} LiDAR frames")

# 3. Cleanup.
device.stop()
device.disconnect()
```

### Inline Snippet (in Markdown)

Use fenced code blocks with language annotation and a title comment:

````markdown
```cpp
// Connect to a real device over Ethernet.
thunderbird::DeviceConfig config;
config.uri = "eth://192.168.1.100:7500";

thunderbird::DeviceManager device(config);
auto status = device.connect();          // blocks until handshake completes
```
````
