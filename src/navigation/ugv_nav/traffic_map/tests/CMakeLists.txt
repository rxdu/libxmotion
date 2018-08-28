# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_traffic_flow test_traffic_flow.cpp)
target_link_libraries(test_traffic_flow trafficmap roadmap tfield trafficviz lightviz utilities)

add_executable(test_traffic_map test_traffic_map.cpp)
target_link_libraries(test_traffic_map trafficmap roadmap tfield trafficviz lightviz utilities)

# add_executable(test_motion_model test_motion_model.cpp)
# target_link_libraries(test_motion_model trafficmap lightviz utilities)

# add_executable(test_threat_ranking test_threat_ranking.cpp)
# target_link_libraries(test_threat_ranking trafficmap roadmap tfield trafficviz lightviz utilities)

# add_executable(test_flow_bt test_flow_bt.cpp)
# target_link_libraries(test_flow_bt trafficmap roadmap tfield trafficviz lightviz utilities)
