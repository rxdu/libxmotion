# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_trafficmap test_trafficmap.cpp)
target_link_libraries(test_trafficmap traffic lightviz ugvnav_viz utils)

add_executable(test_trafficmap2 test_trafficmap2.cpp)
target_link_libraries(test_trafficmap2 traffic lightviz ugvnav_viz utils)