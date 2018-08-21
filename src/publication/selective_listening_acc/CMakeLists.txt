# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add executables
add_executable(show_vehicle_position show_vehicle_position.cpp)
target_link_libraries(show_vehicle_position rank lightviz utilities)

add_executable(pub_topogeo_graph gen_topogeo_graph.cpp)
target_link_libraries(pub_topogeo_graph rank lightviz utilities)

add_executable(pub_threat_ranking_case1 threat_ranking_case1.cpp)
target_link_libraries(pub_threat_ranking_case1 rank roadmap tfield trafficviz lightviz utilities)

add_executable(pub_threat_ranking_case2 threat_ranking_case2.cpp)
target_link_libraries(pub_threat_ranking_case2 rank roadmap tfield trafficviz lightviz utilities)
