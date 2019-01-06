# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_vehicle_draw test_vehicle_draw.cpp)
target_link_libraries(test_vehicle_draw navviz utils)

add_executable(test_roadmap_viz test_roadmap_viz.cpp)
target_link_libraries(test_roadmap_viz navviz utils)

add_executable(test_lattice_viz test_lattice_viz.cpp)
target_link_libraries(test_lattice_viz navviz utils)

add_executable(test_static_threat_viz test_static_threat_viz.cpp)
target_link_libraries(test_static_threat_viz navviz utils)

# add_executable(test_traffic_viz test_traffic_viz.cpp)
# target_link_libraries(test_traffic_viz tfield navviz lightviz utils)