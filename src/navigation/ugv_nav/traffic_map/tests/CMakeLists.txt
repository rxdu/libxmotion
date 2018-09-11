# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_traffic_flow test_traffic_flow.cpp)
target_link_libraries(test_traffic_flow trafficmap roadmap tfield trafficviz lightviz utilities)

add_executable(test_traffic_map test_traffic_map.cpp)
target_link_libraries(test_traffic_map trafficmap roadmap tfield trafficviz lightviz utilities)

add_executable(test_traffic_map2 test_traffic_map2.cpp)
target_link_libraries(test_traffic_map2 trafficmap roadmap tfield trafficviz lightviz utilities)

add_executable(test_flow_bt test_flow_bt.cpp)
target_link_libraries(test_flow_bt cthreat trafficmap roadmap tfield trafficviz lightviz utilities)