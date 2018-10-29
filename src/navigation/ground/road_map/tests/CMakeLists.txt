# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_roadmap_internal test_roadmap_internal.cpp)
target_link_libraries(test_roadmap_internal roadmap lightviz utilities)

add_executable(test_roadmap test_roadmap.cpp)
target_link_libraries(test_roadmap roadmap lightviz ugvnav_viz utilities)

add_executable(test_roadmap2 test_roadmap2.cpp)
target_link_libraries(test_roadmap2 roadmap lightviz ugvnav_viz utilities)

add_executable(test_lane_collision test_lane_collision.cpp)
target_link_libraries(test_lane_collision roadmap lightviz utilities)

add_executable(test_topogeo_graph test_topogeo_graph.cpp)
target_link_libraries(test_topogeo_graph roadmap lightviz utilities)