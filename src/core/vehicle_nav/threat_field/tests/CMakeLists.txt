# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_scalar_field test_scalar_field.cpp)
target_link_libraries(test_scalar_field tfield)

# add_executable(test_threat_basis test_threat_basis.cpp)
# target_link_libraries(test_threat_basis tfield fastplot utilities)

# add_executable(test_traffic_participant test_traffic_participant.cpp)
# target_link_libraries(test_traffic_participant tfield fastplot utilities)

add_executable(test_collision_field test_collision_field.cpp)
target_link_libraries(test_collision_field tfield lightviz utilities)

# add_executable(test_info_field test_info_field.cpp)
# target_link_libraries(test_info_field tfield fastplot utilities)

# add_executable(test_coeff_est test_coeff_est.cpp)
# target_link_libraries(test_coeff_est tfield fastplot utilities)

# add_executable(test_create_field test_create_field.cpp)
# target_link_libraries(test_create_field tfield)

# add_executable(test_road_field test_road_field.cpp)
# target_link_libraries(test_road_field tfield fastplot ${LCM_LIBRARIES}) 

# add_executable(test_vehicle_field test_vehicle_field.cpp)
# target_link_libraries(test_vehicle_field tfield fastplot ${LCM_LIBRARIES}) 