find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# Try to find pybind11 via cmake config, then fallback to pip location
find_package(pybind11 QUIET)
if(NOT pybind11_FOUND)
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -m pybind11 --cmakedir
        OUTPUT_VARIABLE pybind11_DIR
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    find_package(pybind11 REQUIRED)
endif()

pybind11_add_module(thunderbird_py
    src/bindings.cpp
)

target_link_libraries(thunderbird_py PRIVATE thunderbird_sdk)

# Install next to the SDK or into site-packages
install(TARGETS thunderbird_py
    DESTINATION ${Python3_SITEARCH}
)
