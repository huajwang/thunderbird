cmake_minimum_required(VERSION 3.16)
project(thunderbird_firmware C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

option(FW_CROSS_COMPILE "Cross-compiling for bare-metal (skip TCP transport and tests)" OFF)
option(FW_TCP_ONLY "Route all traffic over TCP instead of TCP+UDP dual-channel" OFF)

# ─── Core library (platform-independent) ─────────────────────────────────────

set(FW_CORE_SOURCES
    src/protocol.c
    src/platform_clock.c
    src/control.c
    src/stub_lidar.c
    src/stub_imu.c
    src/stub_camera.c
)

# ─── Full desktop firmware (native builds only) ─────────────────────────────

if(NOT FW_CROSS_COMPILE)
    add_executable(thunderbird_firmware
        src/main.c
        ${FW_CORE_SOURCES}
        src/transport_tcp.c
        src/mux.c
    )
    target_include_directories(thunderbird_firmware PRIVATE include)

    if(FW_TCP_ONLY)
        target_compile_definitions(thunderbird_firmware PRIVATE FW_TCP_ONLY)
    endif()

    if(WIN32)
        target_link_libraries(thunderbird_firmware PRIVATE ws2_32)
    else()
        target_link_libraries(thunderbird_firmware PRIVATE m)
    endif()
endif()

# ─── Static library for cross-compile verification ──────────────────────────

if(FW_CROSS_COMPILE)
    add_library(thunderbird_fw_core STATIC ${FW_CORE_SOURCES})
    target_include_directories(thunderbird_fw_core PUBLIC include)
    target_compile_definitions(thunderbird_fw_core PRIVATE FW_BARE_METAL)
endif()

# ─── Tests (native builds only) ─────────────────────────────────────────────

if(NOT FW_CROSS_COMPILE)
    enable_testing()

    # Protocol & CRC tests
    add_executable(test_protocol tests/test_protocol.c ${FW_CORE_SOURCES})
    target_include_directories(test_protocol PRIVATE include)
    target_compile_options(test_protocol PRIVATE -UNDEBUG)
    if(WIN32)
        target_link_libraries(test_protocol PRIVATE ws2_32)
    else()
        target_link_libraries(test_protocol PRIVATE m)
    endif()
    add_test(NAME protocol_tests COMMAND test_protocol)

    # Control handler tests
    add_executable(test_control tests/test_control.c ${FW_CORE_SOURCES})
    target_include_directories(test_control PRIVATE include)
    target_compile_options(test_control PRIVATE -UNDEBUG)
    if(WIN32)
        target_link_libraries(test_control PRIVATE ws2_32)
    else()
        target_link_libraries(test_control PRIVATE m)
    endif()
    add_test(NAME control_tests COMMAND test_control)

    # Stub sensor driver tests
    add_executable(test_sensors tests/test_sensors.c ${FW_CORE_SOURCES})
    target_include_directories(test_sensors PRIVATE include)
    target_compile_options(test_sensors PRIVATE -UNDEBUG)
    if(WIN32)
        target_link_libraries(test_sensors PRIVATE ws2_32)
    else()
        target_link_libraries(test_sensors PRIVATE m)
    endif()
    add_test(NAME sensor_tests COMMAND test_sensors)
endif()
