# ──────────────────────────────────────────────────────────────────────────────
# Thunderbird SDK — slam_eval: SLAM Evaluation Framework
# ──────────────────────────────────────────────────────────────────────────────

cmake_minimum_required(VERSION 3.16)

# ── Library: libslam_eval (adapters + evaluator + utils) ─────────────────────
add_library(slam_eval STATIC
    src/adapters/kitti_adapter.cpp
    src/adapters/euroc_adapter.cpp
    src/adapters/stress_test_adapter.cpp
    src/adapter_factory.cpp
    src/evaluator.cpp
    src/fault_injector.cpp
    src/frame_stream.cpp
    src/imu_interpolator.cpp
    src/perf_timer.cpp
    src/pose_recorder.cpp
    src/resource_monitor.cpp
    src/trajectory_alignment.cpp
    src/trajectory_io.cpp
)

target_include_directories(slam_eval
    PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(slam_eval
    PUBLIC  thunderbird_sdk_shared
    PRIVATE Threads::Threads
)

target_compile_features(slam_eval PUBLIC cxx_std_20)

# ── CLI executable: slam_eval ────────────────────────────────────────────────
add_executable(slam_eval_cli src/main.cpp)

set_target_properties(slam_eval_cli PROPERTIES
    OUTPUT_NAME slam_eval
)

target_link_libraries(slam_eval_cli
    PRIVATE slam_eval
    PRIVATE Threads::Threads
)

target_compile_definitions(slam_eval_cli PRIVATE
    EVAL_SCRIPTS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/scripts"
)

target_compile_features(slam_eval_cli PRIVATE cxx_std_20)

# ── Test: trajectory alignment sanity ────────────────────────────────────
add_executable(test_trajectory_alignment tests/test_trajectory_alignment.cpp)
target_link_libraries(test_trajectory_alignment PRIVATE slam_eval)
target_compile_features(test_trajectory_alignment PRIVATE cxx_std_20)
add_test(NAME TrajectoryAlignment COMMAND test_trajectory_alignment)
