# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_rm test_roadmap.cpp)
target_link_libraries(test_rm roadmap lightviz utilities)

add_executable(test_lane_collision test_lane_collision.cpp)
target_link_libraries(test_lane_collision roadmap lightviz utilities)