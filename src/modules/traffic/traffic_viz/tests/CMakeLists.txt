# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_vehicle_draw test_vehicle_draw.cpp)
target_link_libraries(test_vehicle_draw trafficviz utils)

add_executable(test_roadmap_viz test_roadmap_viz.cpp)
target_link_libraries(test_roadmap_viz trafficviz utils)

add_executable(test_lattice_viz test_lattice_viz.cpp)
target_link_libraries(test_lattice_viz trafficviz utils)

add_executable(test_static_threat_viz test_static_threat_viz.cpp)
target_link_libraries(test_static_threat_viz trafficviz utils)

add_executable(test_lattice_graph_viz test_lattice_graph_viz.cpp)
target_link_libraries(test_lattice_graph_viz trafficviz utils)

# add_executable(test_lookahead_zone_viz test_lookahead_zone_viz.cpp)
# target_link_libraries(test_lookahead_zone_viz trafficviz utils)

# add_executable(test_threat_field_viz test_threat_field_viz.cpp)
# target_link_libraries(test_threat_field_viz trafficviz utils)

add_executable(test_lattice_threat_viz test_lattice_threat_viz.cpp)
target_link_libraries(test_lattice_threat_viz trafficviz utils)

# add_executable(test_traffic_viz test_traffic_viz.cpp)
# target_link_libraries(test_traffic_viz threatdist trafficviz utils)