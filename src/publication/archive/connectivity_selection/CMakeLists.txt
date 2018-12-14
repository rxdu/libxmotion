# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add executables
# add_executable(gen_true_threat gen_true_threat.cpp)
# target_link_libraries(gen_true_threat tfield ugvnav_viz lightviz utils)

# add_executable(show_vehicle_position show_vehicle_position.cpp)
# target_link_libraries(show_vehicle_position cthreat roadmap trafficmap lightviz utils)

add_executable(pub_topogeo_graph pub_topogeo_graph.cpp)
target_link_libraries(pub_topogeo_graph roadmap lightviz utils)

add_executable(pub_topogeo_roundabout pub_topogeo_roundabout.cpp)
target_link_libraries(pub_topogeo_roundabout roadmap lightviz utils)

add_executable(pub_roadmap_grid pub_roadmap_grid.cpp)
target_link_libraries(pub_roadmap_grid traffic ugvnav_viz)

add_executable(pub_lane_occupancy pub_lane_occupancy.cpp)
target_link_libraries(pub_lane_occupancy traffic ugvnav_viz)

add_executable(pub_vehicle_threat pub_vehicle_threat.cpp)
target_link_libraries(pub_vehicle_threat traffic ugvnav_viz)

add_executable(pub_lattice_graph pub_lattice_graph.cpp)
target_link_libraries(pub_lattice_graph lattice local_planner ugvnav_viz utils)

add_executable(pub_evaluation1 pub_evaluation1.cpp)
target_link_libraries(pub_evaluation1 connectivity ugvnav_viz utils)

add_executable(pub_evaluation2 pub_evaluation2.cpp)
target_link_libraries(pub_evaluation2 connectivity ugvnav_viz utils)

add_executable(pub_evaluation3 pub_evaluation3.cpp)
target_link_libraries(pub_evaluation3 connectivity ugvnav_viz utils)

# add_executable(pub_planning_true_field planning_true_field.cpp)
# target_link_libraries(pub_planning_true_field cthreat roadmap trafficmap roadmap tfield ugvnav_viz lightviz utils)

