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
