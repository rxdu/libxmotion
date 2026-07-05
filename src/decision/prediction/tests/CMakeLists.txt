# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
# add_executable(test_traffic_participant test_traffic_participant.cpp)
# target_link_libraries(test_traffic_participant threatdist trafficviz utilities)

# add_executable(test_collision_field test_collision_field.cpp)
# target_link_libraries(test_collision_field threatdist trafficviz utilities)

# add_executable(test_true_threat test_true_threat.cpp)
# target_link_libraries(test_true_threat threatdist trafficviz utilities)

################### 

add_executable(test_static_threat test_static_threat.cpp)
target_link_libraries(test_static_threat threatdist trafficviz utilities)

add_executable(test_occupancy_distribution test_occupancy_distribution.cpp)
target_link_libraries(test_occupancy_distribution threatdist trafficviz utilities)

add_executable(test_occupancy_interval test_occupancy_interval.cpp)
target_link_libraries(test_occupancy_interval threatdist trafficviz utilities)

add_executable(test_gaussian_cthreat test_gaussian_cthreat.cpp)
target_link_libraries(test_gaussian_cthreat threatdist trafficviz utilities)

add_executable(test_threat_field test_threat_field.cpp)
target_link_libraries(test_threat_field threatdist trafficviz utilities)

add_executable(test_threat_query test_threat_query.cpp)
target_link_libraries(test_threat_query threatdist trafficviz utilities)

add_executable(test_vehicle_threat test_vehicle_threat.cpp)
target_link_libraries(test_vehicle_threat threatdist utilities)
