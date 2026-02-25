# ── SDK core static library ──────────────────────────────────────────────────
file(GLOB_RECURSE SDK_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

add_library(thunderbird_sdk STATIC ${SDK_SOURCES})

target_include_directories(thunderbird_sdk
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(thunderbird_sdk
    PUBLIC  Threads::Threads
)

# Export compile definitions for simulated mode
if(THUNDERBIRD_USE_SIMULATED)
    target_compile_definitions(thunderbird_sdk PUBLIC THUNDERBIRD_SIMULATED=1)
endif()
