# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/src)

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_scalar_field test_scalar_field.cpp)
target_link_libraries(test_scalar_field field)

# add_executable(test_threat_basis test_threat_basis.cpp)
# target_link_libraries(test_threat_basis field fastplot utilities)

# add_executable(test_traffic_participant test_traffic_participant.cpp)
# target_link_libraries(test_traffic_participant field fastplot utilities)

add_executable(test_collision_field test_collision_field.cpp)
target_link_libraries(test_collision_field field lightviz utilities)

# add_executable(test_info_field test_info_field.cpp)
# target_link_libraries(test_info_field field fastplot utilities)

# add_executable(test_coeff_est test_coeff_est.cpp)
# target_link_libraries(test_coeff_est field fastplot utilities)

# add_executable(test_create_field test_create_field.cpp)
# target_link_libraries(test_create_field field)

# add_executable(test_road_field test_road_field.cpp)
# target_link_libraries(test_road_field field fastplot ${LCM_LIBRARIES}) 

# add_executable(test_vehicle_field test_vehicle_field.cpp)
# target_link_libraries(test_vehicle_field field fastplot ${LCM_LIBRARIES}) 