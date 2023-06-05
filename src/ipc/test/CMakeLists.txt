add_executable(test_lcm_publisher lcm_publisher.cpp)
target_link_libraries(test_lcm_publisher ${LCM_NAMESPACE}lcm ipc_lcmtypes-cpp)

add_executable(test_lcm_subscriber lcm_subscriber.cpp)
target_link_libraries(test_lcm_subscriber ${LCM_NAMESPACE}lcm ipc_lcmtypes-cpp)