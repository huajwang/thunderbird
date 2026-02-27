cmake_minimum_required(VERSION 3.16)
project(thunderbird_sdk VERSION 0.2.0 LANGUAGES CXX)

# ── Version override (CI injects from Git tag) ──────────────────────────────
# Usage: cmake -DTHUNDERBIRD_VERSION_OVERRIDE=1.2.3 ..
# If set, overrides the project(VERSION) for all downstream consumers.
if(DEFINED THUNDERBIRD_VERSION_OVERRIDE)
    # Validate format
    if(NOT THUNDERBIRD_VERSION_OVERRIDE MATCHES "^[0-9]+\\.[0-9]+\\.[0-9]+$")
        message(FATAL_ERROR "THUNDERBIRD_VERSION_OVERRIDE must be X.Y.Z, got: ${THUNDERBIRD_VERSION_OVERRIDE}")
    endif()
    set(PROJECT_VERSION "${THUNDERBIRD_VERSION_OVERRIDE}")
    string(REGEX MATCH "^([0-9]+)\\.([0-9]+)\\.([0-9]+)$" _ver_match "${THUNDERBIRD_VERSION_OVERRIDE}")
    set(PROJECT_VERSION_MAJOR "${CMAKE_MATCH_1}")
    set(PROJECT_VERSION_MINOR "${CMAKE_MATCH_2}")
    set(PROJECT_VERSION_PATCH "${CMAKE_MATCH_3}")
    set(thunderbird_sdk_VERSION "${PROJECT_VERSION}")
    set(thunderbird_sdk_VERSION_MAJOR "${PROJECT_VERSION_MAJOR}")
    set(thunderbird_sdk_VERSION_MINOR "${PROJECT_VERSION_MINOR}")
    set(thunderbird_sdk_VERSION_PATCH "${PROJECT_VERSION_PATCH}")
    message(STATUS "Version override: ${PROJECT_VERSION} (from THUNDERBIRD_VERSION_OVERRIDE)")
else()
    message(STATUS "Version: ${PROJECT_VERSION} (from project() directive)")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── Options ──────────────────────────────────────────────────────────────────
option(THUNDERBIRD_BUILD_EXAMPLES  "Build example applications"         ON)
option(THUNDERBIRD_BUILD_TESTS     "Build unit tests"                   ON)
option(THUNDERBIRD_BUILD_PYTHON    "Build Python bindings (pybind11)"   OFF)
option(THUNDERBIRD_BUILD_ROS2      "Build ROS 2 bridge package"         OFF)
option(THUNDERBIRD_BUILD_SLAMD     "Build SLAM daemon (acme_slamd)"     OFF)
option(THUNDERBIRD_BUILD_EVAL      "Build SLAM evaluation framework"    OFF)
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

# ── SLAM daemon ─────────────────────────────────────────────────────────────
if(THUNDERBIRD_BUILD_SLAMD)
    add_subdirectory(slamd)
endif()

# ── SLAM evaluation framework ──────────────────────────────────────────────
if(THUNDERBIRD_BUILD_EVAL)
    add_subdirectory(eval)
endif()

# ── Tests ────────────────────────────────────────────────────────────────────
if(THUNDERBIRD_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
