enable_testing()

add_executable(test_ring_buffer  test_ring_buffer.cpp)
target_link_libraries(test_ring_buffer PRIVATE thunderbird_sdk)
add_test(NAME RingBuffer COMMAND test_ring_buffer)

add_executable(test_sync_engine  test_sync_engine.cpp)
target_link_libraries(test_sync_engine PRIVATE thunderbird_sdk)
add_test(NAME SyncEngine COMMAND test_sync_engine)

add_executable(test_device_manager test_device_manager.cpp)
target_link_libraries(test_device_manager PRIVATE thunderbird_sdk)
add_test(NAME DeviceManager COMMAND test_device_manager)

add_executable(test_protocol_parser test_protocol_parser.cpp)
target_link_libraries(test_protocol_parser PRIVATE thunderbird_sdk)
add_test(NAME ProtocolParser COMMAND test_protocol_parser)

add_executable(test_data_layer test_data_layer.cpp)
target_link_libraries(test_data_layer PRIVATE thunderbird_sdk)
add_test(NAME DataLayer COMMAND test_data_layer)

add_executable(test_time_sync test_time_sync.cpp)
target_link_libraries(test_time_sync PRIVATE thunderbird_sdk)
add_test(NAME TimeSync COMMAND test_time_sync)

add_executable(test_ros2_bridge test_ros2_bridge.cpp)
target_link_libraries(test_ros2_bridge PRIVATE thunderbird_sdk)
add_test(NAME Ros2Helpers COMMAND test_ros2_bridge)

add_executable(test_recorder test_recorder.cpp)
target_link_libraries(test_recorder PRIVATE thunderbird_sdk)
add_test(NAME Recorder COMMAND test_recorder)

add_executable(test_slam_benchmark test_slam_benchmark.cpp)
target_link_libraries(test_slam_benchmark PRIVATE thunderbird_sdk Threads::Threads)
target_compile_definitions(test_slam_benchmark PRIVATE THUNDERBIRD_SLAM_PROFILING=1)
add_test(NAME SlamBenchmark COMMAND test_slam_benchmark --no-plans)

add_executable(test_slam_health test_slam_health.cpp)
target_link_libraries(test_slam_health PRIVATE thunderbird_sdk)
add_test(NAME SlamHealth COMMAND test_slam_health)

add_executable(test_runtime_profiler test_runtime_profiler.cpp)
target_link_libraries(test_runtime_profiler PRIVATE thunderbird_sdk Threads::Threads)
add_test(NAME RuntimeProfiler COMMAND test_runtime_profiler)

add_executable(test_object_detector test_object_detector.cpp)
target_link_libraries(test_object_detector PRIVATE thunderbird_sdk)
add_test(NAME ObjectDetector COMMAND test_object_detector)

add_executable(test_multi_object_tracker test_multi_object_tracker.cpp)
target_link_libraries(test_multi_object_tracker PRIVATE thunderbird_sdk)
add_test(NAME MultiObjectTracker COMMAND test_multi_object_tracker)

add_executable(test_preprocessor test_preprocessor.cpp)
target_link_libraries(test_preprocessor PRIVATE thunderbird_sdk)
add_test(NAME Preprocessor COMMAND test_preprocessor)

add_executable(test_perception_engine test_perception_engine.cpp)
target_link_libraries(test_perception_engine PRIVATE thunderbird_sdk Threads::Threads)
add_test(NAME PerceptionEngine COMMAND test_perception_engine)

add_executable(test_kalman_hungarian test_kalman_hungarian.cpp)
target_include_directories(test_kalman_hungarian PRIVATE ${CMAKE_SOURCE_DIR}/sdk/src/perception)
target_link_libraries(test_kalman_hungarian PRIVATE thunderbird_sdk)
add_test(NAME KalmanHungarian COMMAND test_kalman_hungarian)

add_executable(test_device_health_monitor test_device_health_monitor.cpp)
target_link_libraries(test_device_health_monitor PRIVATE thunderbird_sdk Threads::Threads)
add_test(NAME DeviceHealthMonitor COMMAND test_device_health_monitor)

add_executable(test_phase2_hw_integration test_phase2_hw_integration.cpp)
target_link_libraries(test_phase2_hw_integration PRIVATE thunderbird_sdk)
add_test(NAME Phase2HwIntegration COMMAND test_phase2_hw_integration)

add_executable(test_logging test_logging.cpp)
target_link_libraries(test_logging PRIVATE thunderbird_sdk)
add_test(NAME Logging COMMAND test_logging)

add_executable(test_diagnostics test_diagnostics.cpp)
target_link_libraries(test_diagnostics PRIVATE thunderbird_sdk Threads::Threads)
add_test(NAME Diagnostics COMMAND test_diagnostics)

# ── Calibration / Phase 1-8 tests ──────────────────────────────────────────

add_executable(test_calibration_types test_calibration_types.cpp)
target_link_libraries(test_calibration_types PRIVATE thunderbird_sdk)
add_test(NAME CalibrationTypes COMMAND test_calibration_types)

add_executable(test_bspline test_bspline.cpp)
target_include_directories(test_bspline PRIVATE ${CMAKE_SOURCE_DIR}/sdk/src)
target_link_libraries(test_bspline PRIVATE thunderbird_sdk)
add_test(NAME BSpline COMMAND test_bspline)

add_executable(test_rigid_transform test_rigid_transform.cpp)
target_include_directories(test_rigid_transform PRIVATE ${CMAKE_SOURCE_DIR}/sdk/src)
target_link_libraries(test_rigid_transform PRIVATE thunderbird_sdk)
add_test(NAME RigidTransform COMMAND test_rigid_transform)

add_executable(test_online_refiner test_online_refiner.cpp)
target_include_directories(test_online_refiner PRIVATE ${CMAKE_SOURCE_DIR}/sdk/src)
target_link_libraries(test_online_refiner PRIVATE thunderbird_sdk)
add_test(NAME OnlineRefiner COMMAND test_online_refiner)

add_executable(test_imu_interpolation test_imu_interpolation.cpp)
target_link_libraries(test_imu_interpolation PRIVATE thunderbird_sdk)
add_test(NAME ImuInterpolation COMMAND test_imu_interpolation)

add_executable(test_clock_service_seed test_clock_service_seed.cpp)
target_link_libraries(test_clock_service_seed PRIVATE thunderbird_sdk)
add_test(NAME ClockServiceSeed COMMAND test_clock_service_seed)

# Python converter tests
find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_FOUND)
    add_test(NAME TbrecConverters
             COMMAND ${Python3_EXECUTABLE} test_tbrec_converters.py
             WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
endif()
