# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_cfield_viz test_cfield_viz.cpp)
target_link_libraries(test_cfield_viz tfield trafficviz lightviz utilities)

add_executable(test_roadmap_viz test_roadmap_viz.cpp)
target_link_libraries(test_roadmap_viz tfield trafficviz lightviz utilities)
