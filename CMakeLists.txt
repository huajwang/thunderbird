cmake_minimum_required(VERSION 3.16)
project(thunderbird_sdk VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── Options ──────────────────────────────────────────────────────────────────
option(THUNDERBIRD_BUILD_EXAMPLES  "Build example applications"         ON)
option(THUNDERBIRD_BUILD_TESTS     "Build unit tests"                   ON)
option(THUNDERBIRD_BUILD_PYTHON    "Build Python bindings (pybind11)"   OFF)
option(THUNDERBIRD_BUILD_ROS2      "Build ROS 2 bridge package"         OFF)
option(THUNDERBIRD_USE_SIMULATED   "Use simulated sensor backends"      ON)

# ── Compiler warnings ───────────────────────────────────────────────────────
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ─────────────────────────────────────────────────────────────
find_package(Threads REQUIRED)

# ── SDK core library ─────────────────────────────────────────────────────────
add_subdirectory(sdk)

# ── Examples ─────────────────────────────────────────────────────────────────
if(THUNDERBIRD_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# ── Python bindings ──────────────────────────────────────────────────────────
if(THUNDERBIRD_BUILD_PYTHON)
    add_subdirectory(sdk/python)
endif()

# ── ROS 2 bridge ────────────────────────────────────────────────────────────
if(THUNDERBIRD_BUILD_ROS2)
    add_subdirectory(ros2_bridge)
endif()

# ── Tests ────────────────────────────────────────────────────────────────────
if(THUNDERBIRD_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
