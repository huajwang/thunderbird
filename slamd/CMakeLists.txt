# ─────────────────────────────────────────────────────────────────────────────
# acme_slamd — CMake build
# ─────────────────────────────────────────────────────────────────────────────

# ── Option to build the SLAM daemon ──────────────────────────────────────────
option(SLAMD_ENABLE_ZMQ     "Link against libzmq for IPC notifications" OFF)
option(SLAMD_ENABLE_SYSTEMD "Link against libsystemd for sd_notify"     OFF)
option(SLAMD_ENABLE_YAML    "Link against yaml-cpp for config parsing"  OFF)

# ── Sources ──────────────────────────────────────────────────────────────────
add_executable(acme_slamd
    src/main.cpp
    src/acme_slamd.cpp
)

target_include_directories(acme_slamd
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(acme_slamd
    PRIVATE
        thunderbird_sdk
        Threads::Threads
)

target_compile_features(acme_slamd PRIVATE cxx_std_20)

# ── Version define ───────────────────────────────────────────────────────────
target_compile_definitions(acme_slamd PRIVATE
    SLAMD_VERSION="${PROJECT_VERSION}"
)

# ── Optional: ZeroMQ ─────────────────────────────────────────────────────────
if(SLAMD_ENABLE_ZMQ)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(ZMQ REQUIRED IMPORTED_TARGET libzmq)
    target_link_libraries(acme_slamd PRIVATE PkgConfig::ZMQ)
    target_compile_definitions(acme_slamd PRIVATE SLAMD_HAS_ZMQ=1)
endif()

# ── Optional: libsystemd ────────────────────────────────────────────────────
if(SLAMD_ENABLE_SYSTEMD)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(SYSTEMD REQUIRED IMPORTED_TARGET libsystemd)
    target_link_libraries(acme_slamd PRIVATE PkgConfig::SYSTEMD)
    target_compile_definitions(acme_slamd PRIVATE SLAMD_HAS_SYSTEMD=1)
endif()

# ── Optional: yaml-cpp ──────────────────────────────────────────────────────
if(SLAMD_ENABLE_YAML)
    find_package(yaml-cpp REQUIRED)
    target_link_libraries(acme_slamd PRIVATE yaml-cpp)
    target_compile_definitions(acme_slamd PRIVATE SLAMD_HAS_YAML=1)
endif()

# ── Install rules ────────────────────────────────────────────────────────────
install(TARGETS acme_slamd
    RUNTIME DESTINATION bin
    COMPONENT runtime
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/config/config.yaml
    DESTINATION /etc/acme_slamd
    COMPONENT runtime
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/systemd/acme-slamd.service
    DESTINATION lib/systemd/system
    COMPONENT runtime
)
