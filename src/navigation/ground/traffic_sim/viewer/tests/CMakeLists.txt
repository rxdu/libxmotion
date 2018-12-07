# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_traffic_viewer test_traffic_viewer.cpp)
target_link_libraries(test_traffic_viewer sim_viewer)