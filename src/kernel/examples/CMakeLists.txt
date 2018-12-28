# Add executables
add_executable(lcm_subscriber_cpp src/lcm_subscriber.cpp)
target_link_libraries(lcm_subscriber_cpp datalink)

add_executable(lcm_publisher_cpp src/lcm_publisher.cpp)
target_link_libraries(lcm_publisher_cpp datalink)

add_executable(logger_example src/logger_example.cpp)
target_link_libraries(logger_example utils)

# add_executable(stopwatch_example src/stopwatch_example.cpp)
# target_link_libraries(stopwatch_example utils)

add_executable(simple_graph_example src/simple_graph_example.cpp)
target_link_libraries(simple_graph_example graph)

