# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_roadmap test_roadmap.cpp)
target_link_libraries(test_roadmap roadmap lightviz utilities)

add_executable(test_lane_collision test_lane_collision.cpp)
target_link_libraries(test_lane_collision roadmap lightviz utilities)