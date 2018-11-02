# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_trafficmap test_trafficmap.cpp)
target_link_libraries(test_trafficmap traffic lightviz ugvnav_viz utilities)

add_executable(test_lane_collision test_lane_collision.cpp)
target_link_libraries(test_lane_collision traffic lightviz utilities)