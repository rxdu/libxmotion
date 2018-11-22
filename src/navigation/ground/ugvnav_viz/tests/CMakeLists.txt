# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
# add_executable(test_cfield_viz test_cfield_viz.cpp)
# target_link_libraries(test_cfield_viz tfield ugvnav_viz lightviz utils)

add_executable(test_roadmap_viz test_roadmap_viz.cpp)
target_link_libraries(test_roadmap_viz tfield ugvnav_viz lightviz utils)

add_executable(test_traffic_viz test_traffic_viz.cpp)
target_link_libraries(test_traffic_viz tfield ugvnav_viz lightviz utils)