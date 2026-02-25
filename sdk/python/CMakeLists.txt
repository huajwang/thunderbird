# ── Python bindings (pybind11) ────────────────────────────────────────────────
#
# Builds the `_spatial_sdk_core` C++ extension that is imported by
# spatial_sdk/__init__.py.
#
# Build (Linux — typical):
#   cmake -S . -B build -DTHUNDERBIRD_BUILD_PYTHON=ON \
#         -DTHUNDERBIRD_USE_SIMULATED=ON
#   cmake --build build --target _spatial_sdk_core
#
# Build (cross-platform with manual paths):
#   cmake -S . -B build -DTHUNDERBIRD_BUILD_PYTHON=ON \
#         -DPython3_EXECUTABLE=/path/to/python3 \
#         -DTHUNDERBIRD_USE_SIMULATED=ON
#
# The resulting .so / .pyd lands in the build tree.  Set PYTHONPATH to
# include both:
#   - <build>/sdk/python/          (for the _spatial_sdk_core.so)
#   - <source>/sdk/python/         (for the spatial_sdk/ package)
#
# When built via scikit-build-core (pip install / cibuildwheel), SKBUILD
# is set and the extension is installed into spatial_sdk/ alongside
# __init__.py so the wheel is self-contained.
# ─────────────────────────────────────────────────────────────────────────────

# ── Locate pybind11 ──────────────────────────────────────────────────────────
# Strategy: try find_package first (pip install pybind11 / system package),
# then fall back to FetchContent for a hermetic build.

find_package(pybind11 QUIET)

if(NOT pybind11_FOUND)
    message(STATUS "pybind11 not found locally — fetching via FetchContent")
    include(FetchContent)
    FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG        v2.12.0
    )
    FetchContent_MakeAvailable(pybind11)
endif()

# ── The extension module ─────────────────────────────────────────────────────

pybind11_add_module(_spatial_sdk_core
    spatial_sdk_bindings.cpp
)

target_link_libraries(_spatial_sdk_core
    PRIVATE thunderbird_sdk
)

target_compile_features(_spatial_sdk_core PRIVATE cxx_std_20)

# ── Wheel build: static-link libstdc++ for ABI safety ─────────────────────
# When building via scikit-build-core (cibuildwheel), we statically link
# libstdc++ so the wheel is self-contained and works on any manylinux2014
# host regardless of its system libstdc++ version.
if(SKBUILD)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        target_link_options(_spatial_sdk_core PRIVATE
            -static-libstdc++
            -static-libgcc
        )
        message(STATUS "Wheel build: statically linking libstdc++ and libgcc")
    endif()
endif()

# ── Install rules ────────────────────────────────────────────────────────────
# scikit-build-core sets SKBUILD=ON and only installs COMPONENT=python.
# The extension .so goes inside the spatial_sdk/ package directory so that
# `from spatial_sdk._spatial_sdk_core import *` works in the installed wheel.
if(SKBUILD)
    install(TARGETS _spatial_sdk_core
        LIBRARY DESTINATION spatial_sdk
        COMPONENT python
    )
    message(STATUS "Wheel build: will install _spatial_sdk_core into spatial_sdk/")
else()
    # ── Development workflow ─────────────────────────────────────────────
    # Copy the pure-Python package next to the .so so `import spatial_sdk`
    # works when PYTHONPATH includes the build directory.
    add_custom_command(TARGET _spatial_sdk_core POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/spatial_sdk
            $<TARGET_FILE_DIR:_spatial_sdk_core>/spatial_sdk
        COMMENT "Copying spatial_sdk Python package next to extension module"
    )
endif()
