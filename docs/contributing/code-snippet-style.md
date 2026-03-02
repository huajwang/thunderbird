# Code Snippet Style Guide

Conventions for all code examples in Thunderbird SDK documentation.

---

## Principles

1. **Compilable** — every C++ snippet must compile as-is with the SDK
   installed (unless explicitly marked as pseudocode).
2. **Minimal** — show the shortest path to a working result; omit error
   handling, logging, and CLI parsing unless they are the point.
3. **Self-contained** — include all necessary `#include` directives and
   namespace qualifications.
4. **Consistent** — follow the same naming, formatting, and structure
   across all docs.

---

## C++ Snippet Template

````markdown
```cpp
#include <thunderbird/device_manager.h>    // always show includes
#include <cstdio>

int main() {
    thunderbird::DeviceManager device;     // fully qualified namespace
    device.connect();
    device.start();

    auto frame = device.pullLidar(std::chrono::milliseconds(100));
    if (frame) {
        std::printf("Points: %zu\n", frame->points.size());
    }

    device.stop();
    return 0;
}
```
````

### Rules

| Rule | Example |
|------|---------|
| Fully qualify namespaces | `thunderbird::data::LidarFrame`, not `LidarFrame` |
| Use `std::printf` over `std::cout` | Shorter, no `<iostream>` bloat |
| Use `auto` for return types | `auto frame = device.pullLidar(...)` |
| Use chrono literals where clear | `std::chrono::milliseconds(100)` |
| No `using namespace` | Always qualify |
| 4-space indent | Match project `.clang-format` |
| Max 40 lines | Split longer examples into stages |
| Mark omissions with `// ...` | Never use `/* ... */` or prose inside code |

---

## Python Snippet Template

````markdown
```python
import spatial_sdk as tb

device = tb.DeviceManager()
device.connect()
device.start()

frame = device.pull_lidar(timeout_ms=100)
if frame:
    pts = frame.points()  # numpy (N, 4) float32
    print(f"Points: {len(pts)}")

device.stop()
```
````

### Rules

| Rule | Example |
|------|---------|
| Import as `tb` | `import spatial_sdk as tb` |
| Snake_case method names | `pull_lidar`, not `pullLidar` |
| Type hints on function defs | `def process(frame: tb.LidarFrame) -> None:` |
| Max 30 lines | |

---

## Inline Markdown Snippets

For inline references to API symbols in prose:

| Context | Format | Example |
|---------|--------|---------|
| Class | `` `thunderbird::DeviceManager` `` | `thunderbird::DeviceManager` |
| Method | `` `device.connect()` `` | `device.connect()` |
| Parameter | `` `config.retry.max_retries` `` | `config.retry.max_retries` |
| File | `` `device_manager.h` `` (with link) | [device_manager.h](../sdk/include/thunderbird/device_manager.h) |
| CLI command | `` `cmake --build build` `` | `cmake --build build` |

---

## Build Instructions for Snippets

When a snippet is meant to be compiled standalone, include build
instructions immediately below:

````markdown
**Build:**

```bash
g++ -std=c++20 -o example example.cpp \
    -I/usr/local/include \
    -lthunderbird_sdk -lpthread
```

Or with CMake:

```cmake
find_package(ThunderbirdSDK REQUIRED)
add_executable(example example.cpp)
target_link_libraries(example Thunderbird::SDK)
```
````

---

## Language Fence Tags

Always use a language tag on fenced code blocks:

| Language | Tag |
|----------|-----|
| C++ | `cpp` |
| Python | `python` |
| Bash / shell | `bash` |
| CMake | `cmake` |
| JSON | `json` |
| YAML | `yaml` |
| Plain text / output | `text` |

Never use bare ` ``` ` without a language tag.

---

## Output Blocks

Show expected output in a separate `text` block:

````markdown
**Output:**

```text
Points: 28800
Synced frame: 28800 pts, 20 IMUs, camera: yes
```
````

---

## Anti-Patterns

| Don't | Do Instead |
|-------|-----------|
| `using namespace thunderbird;` | `thunderbird::DeviceManager` |
| `std::cout << "Points: " << n << std::endl;` | `std::printf("Points: %zu\n", n);` |
| Snippets that require 10+ includes | Show only what's needed |
| Pseudocode without marking it | Add `// pseudocode` comment |
| Copy-paste of full source files | Link to `examples/` and show only the key section |
