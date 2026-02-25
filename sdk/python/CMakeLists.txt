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

# Copy the pure-Python package next to the .so so `import spatial_sdk` works
# when PYTHONPATH includes the build directory.
add_custom_command(TARGET _spatial_sdk_core POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/spatial_sdk
        $<TARGET_FILE_DIR:_spatial_sdk_core>/spatial_sdk
    COMMENT "Copying spatial_sdk Python package next to extension module"
)
