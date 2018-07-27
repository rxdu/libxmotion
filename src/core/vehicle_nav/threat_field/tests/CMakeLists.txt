# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_scalar_field test_scalar_field.cpp)
target_link_libraries(test_scalar_field tfield)

add_executable(test_threat_basis test_threat_basis.cpp)
target_link_libraries(test_threat_basis tfield lightviz utilities)

add_executable(test_traffic_participant test_traffic_participant.cpp)
target_link_libraries(test_traffic_participant tfield lightviz utilities)

add_executable(test_collision_field test_collision_field.cpp)
target_link_libraries(test_collision_field tfield lightviz utilities)
