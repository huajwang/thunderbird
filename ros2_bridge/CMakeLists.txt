cmake_minimum_required(VERSION 3.16)

find_package(ament_cmake REQUIRED)
find_package(rclcpp      REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs    REQUIRED)

# ── Ros2Bridge shared library ───────────────────────────────────────────────
#
# Built as a separate library so multiple executables (the default node,
# the pull-mode demo, user-defined nodes) can link against it without
# duplicating compilation.

add_library(thunderbird_ros2_bridge_lib
    src/ros2_bridge.cpp
)

target_include_directories(thunderbird_ros2_bridge_lib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(thunderbird_ros2_bridge_lib
    PUBLIC thunderbird_sdk
)

ament_target_dependencies(thunderbird_ros2_bridge_lib
    PUBLIC rclcpp sensor_msgs std_msgs
)

# ── Default node executable (callback-driven) ───────────────────────────────

add_executable(thunderbird_ros2_node
    src/thunderbird_ros2_node.cpp
)

target_link_libraries(thunderbird_ros2_node
    PRIVATE thunderbird_ros2_bridge_lib
)

ament_target_dependencies(thunderbird_ros2_node
    rclcpp
)

# ── Pull-mode demo executable ───────────────────────────────────────────────

add_executable(ros2_bridge_pull_demo
    src/ros2_bridge_pull_demo.cpp
)

target_link_libraries(ros2_bridge_pull_demo
    PRIVATE thunderbird_ros2_bridge_lib
)

ament_target_dependencies(ros2_bridge_pull_demo
    rclcpp
)

# ── Install ─────────────────────────────────────────────────────────────────

install(TARGETS
    thunderbird_ros2_bridge_lib
    thunderbird_ros2_node
    ros2_bridge_pull_demo
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

ament_package()
