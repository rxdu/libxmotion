# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add executables
# add_executable(gen_true_threat gen_true_threat.cpp)
# target_link_libraries(gen_true_threat tfield ugvnav_viz lightviz utilities)

# add_executable(show_vehicle_position show_vehicle_position.cpp)
# target_link_libraries(show_vehicle_position cthreat roadmap trafficmap lightviz utilities)

add_executable(pub_topogeo_graph pub_topogeo_graph.cpp)
target_link_libraries(pub_topogeo_graph roadmap lightviz utilities)

add_executable(pub_topogeo_roundabout pub_topogeo_roundabout.cpp)
target_link_libraries(pub_topogeo_roundabout roadmap lightviz utilities)

add_executable(pub_roadmap_grid pub_roadmap_grid.cpp)
target_link_libraries(pub_roadmap_grid traffic ugvnav_viz)

add_executable(pub_lane_occupancy pub_lane_occupancy.cpp)
target_link_libraries(pub_lane_occupancy traffic ugvnav_viz)

add_executable(pub_vehicle_threat pub_vehicle_threat.cpp)
target_link_libraries(pub_vehicle_threat traffic ugvnav_viz)

add_executable(pub_lattice_graph pub_lattice_graph.cpp)
target_link_libraries(pub_lattice_graph lattice local_planner ugvnav_viz utilities)

# add_executable(pub_planning_true_field planning_true_field.cpp)
# target_link_libraries(pub_planning_true_field cthreat roadmap trafficmap roadmap tfield ugvnav_viz lightviz utilities)

