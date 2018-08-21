# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_traffic_participant test_traffic_participant.cpp)
target_link_libraries(test_traffic_participant tfield trafficviz lightviz utilities)

add_executable(test_collision_field test_collision_field.cpp)
target_link_libraries(test_collision_field tfield trafficviz lightviz utilities)
