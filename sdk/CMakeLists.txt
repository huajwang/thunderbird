# ── SDK core libraries (static + shared) ─────────────────────────────────────
file(GLOB_RECURSE SDK_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

# --- Object library (compiled once, linked into both static and shared) ------
add_library(thunderbird_sdk_objects OBJECT ${SDK_SOURCES})
set_target_properties(thunderbird_sdk_objects PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(thunderbird_sdk_objects
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
)
target_link_libraries(thunderbird_sdk_objects PUBLIC Threads::Threads spdlog::spdlog)

# ── Symbol visibility: hide everything by default, export only THUNDERBIRD_API ──
if(NOT MSVC)
    target_compile_options(thunderbird_sdk_objects PRIVATE
        -fvisibility=hidden -fvisibility-inlines-hidden)
endif()
target_compile_definitions(thunderbird_sdk_objects PRIVATE THUNDERBIRD_SDK_EXPORTS)

# ── Compile-time log level stripping ────────────────────────────────────────
# Override with: cmake -DTHUNDERBIRD_LOG_LEVEL_COMPILE=2 ..
# Values: 0=trace 1=debug 2=info 3=warn 4=error 5=critical 6=off
if(DEFINED THUNDERBIRD_LOG_LEVEL_COMPILE)
    target_compile_definitions(thunderbird_sdk_objects PUBLIC
        THUNDERBIRD_LOG_LEVEL_COMPILE=${THUNDERBIRD_LOG_LEVEL_COMPILE})
endif()
if(WIN32)
    target_link_libraries(thunderbird_sdk_objects PUBLIC Psapi)
endif()
if(THUNDERBIRD_USE_SIMULATED)
    target_compile_definitions(thunderbird_sdk_objects PUBLIC THUNDERBIRD_SIMULATED=1)
endif()

# ── Optional GPU perception (PointPillars / CenterPoint via TensorRT) ────────
option(THUNDERBIRD_ENABLE_GPU_PERCEPTION
    "Build GPU-accelerated 3D detector backends (requires CUDA + TensorRT)" OFF)

if(THUNDERBIRD_ENABLE_GPU_PERCEPTION)
    # Guard against older CMake versions where the CUDAToolkit package
    # is not reliably available, even though the project minimum is 3.16.
    if(CMAKE_VERSION VERSION_LESS 3.17)
        message(FATAL_ERROR
            "THUNDERBIRD_ENABLE_GPU_PERCEPTION requires CMake >= 3.17 because it "
            "relies on find_package(CUDAToolkit ...), which is not available in "
            "all CMake 3.16 installations. Your CMake version is ${CMAKE_VERSION}. "
            "Please upgrade CMake or disable THUNDERBIRD_ENABLE_GPU_PERCEPTION.")
    endif()

    # Require a minimum CUDA toolkit version to avoid known-incompatible releases.
    find_package(CUDAToolkit 11.4 REQUIRED)

    # TensorRT doesn't ship a CMake config — locate manually.
    # Set TENSORRT_ROOT to the TensorRT installation prefix if not in standard paths.
    find_path(TENSORRT_INCLUDE_DIR NvInfer.h
        HINTS /usr/include /usr/local/include ${TENSORRT_ROOT}/include)
    find_library(TENSORRT_LIB nvinfer
        HINTS /usr/lib /usr/local/lib ${TENSORRT_ROOT}/lib)

    if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_LIB)
        message(FATAL_ERROR "THUNDERBIRD_ENABLE_GPU_PERCEPTION is ON but TensorRT not found. "
                            "Set TENSORRT_ROOT to the TensorRT installation prefix, or install TensorRT.")
    endif()

    # Verify TensorRT version is at least 8.x (required for EfficientNMS plugin).
    if(EXISTS "${TENSORRT_INCLUDE_DIR}/NvInferVersion.h")
        file(STRINGS "${TENSORRT_INCLUDE_DIR}/NvInferVersion.h" _trt_ver
             REGEX "#define NV_TENSORRT_MAJOR")
        string(REGEX MATCH "[0-9]+" _trt_major "${_trt_ver}")
        if(_trt_major AND _trt_major LESS 8)
            message(FATAL_ERROR "TensorRT >= 8 is required (found major version ${_trt_major}).")
        endif()
    endif()

    target_compile_definitions(thunderbird_sdk_objects PUBLIC THUNDERBIRD_HAS_GPU_PERCEPTION=1)
    target_include_directories(thunderbird_sdk_objects PRIVATE ${TENSORRT_INCLUDE_DIR})
    target_link_libraries(thunderbird_sdk_objects PRIVATE
        ${TENSORRT_LIB}
        CUDA::cudart
    )
    message(STATUS "Thunderbird: GPU perception ENABLED (TensorRT at ${TENSORRT_INCLUDE_DIR})")
else()
    message(STATUS "Thunderbird: GPU perception DISABLED (CpuClusterDetector only)")
endif()

# --- Static library ----------------------------------------------------------
add_library(thunderbird_sdk STATIC $<TARGET_OBJECTS:thunderbird_sdk_objects>)
target_include_directories(thunderbird_sdk
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(thunderbird_sdk PUBLIC Threads::Threads PRIVATE $<BUILD_INTERFACE:spdlog::spdlog>)
target_compile_definitions(thunderbird_sdk INTERFACE THUNDERBIRD_SDK_STATIC)
if(THUNDERBIRD_USE_SIMULATED)
    target_compile_definitions(thunderbird_sdk PUBLIC THUNDERBIRD_SIMULATED=1)
endif()

# --- Shared library ----------------------------------------------------------
set(THUNDERBIRD_SO_VERSION "${PROJECT_VERSION_MAJOR}")

add_library(thunderbird_sdk_shared SHARED $<TARGET_OBJECTS:thunderbird_sdk_objects>)
set_target_properties(thunderbird_sdk_shared PROPERTIES
    OUTPUT_NAME   thunderbird_sdk
    VERSION       ${PROJECT_VERSION}
    SOVERSION     ${THUNDERBIRD_SO_VERSION}
)
target_include_directories(thunderbird_sdk_shared
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(thunderbird_sdk_shared PUBLIC Threads::Threads PRIVATE $<BUILD_INTERFACE:spdlog::spdlog>)
if(THUNDERBIRD_USE_SIMULATED)
    target_compile_definitions(thunderbird_sdk_shared PUBLIC THUNDERBIRD_SIMULATED=1)
endif()

# ── Generate version.h from template ─────────────────────────────────────────
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/thunderbird/version.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/include/thunderbird/version.h
    @ONLY
)

# ── pkg-config file ──────────────────────────────────────────────────────────
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/thunderbird-sdk.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/thunderbird-sdk.pc
    @ONLY
)

# ── CMake package config (for find_package) ──────────────────────────────────
include(CMakePackageConfigHelpers)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/ThunderbirdSDKConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/ThunderbirdSDKConfig.cmake
    INSTALL_DESTINATION lib/cmake/ThunderbirdSDK
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/ThunderbirdSDKConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

# ── Install rules ────────────────────────────────────────────────────────────

# Shared library → runtime package
install(TARGETS thunderbird_sdk_shared
    EXPORT ThunderbirdSDKTargets
    LIBRARY DESTINATION lib/${CMAKE_LIBRARY_ARCHITECTURE}
        COMPONENT runtime
        NAMELINK_SKIP
)

# Static library + shared namelink → dev package
install(TARGETS thunderbird_sdk
    EXPORT ThunderbirdSDKTargets
    ARCHIVE DESTINATION lib/${CMAKE_LIBRARY_ARCHITECTURE}
        COMPONENT dev
)
install(TARGETS thunderbird_sdk_shared
    LIBRARY DESTINATION lib/${CMAKE_LIBRARY_ARCHITECTURE}
        COMPONENT dev
        NAMELINK_ONLY
)

# Headers → dev package
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/thunderbird/
    DESTINATION include/thunderbird
    COMPONENT dev
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/include/thunderbird/version.h
    DESTINATION include/thunderbird
    COMPONENT dev
)

# pkg-config → dev package
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/thunderbird-sdk.pc
    DESTINATION lib/${CMAKE_LIBRARY_ARCHITECTURE}/pkgconfig
    COMPONENT dev
)

# CMake config → dev package
install(EXPORT ThunderbirdSDKTargets
    FILE ThunderbirdSDKTargets.cmake
    NAMESPACE Thunderbird::
    DESTINATION lib/cmake/ThunderbirdSDK
    COMPONENT dev
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/ThunderbirdSDKConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/ThunderbirdSDKConfigVersion.cmake
    DESTINATION lib/cmake/ThunderbirdSDK
    COMPONENT dev
)
