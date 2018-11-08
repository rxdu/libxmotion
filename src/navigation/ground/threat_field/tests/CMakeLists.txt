# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
# add_executable(test_traffic_participant test_traffic_participant.cpp)
# target_link_libraries(test_traffic_participant tfield ugvnav_viz lightviz utilities)

# add_executable(test_collision_field test_collision_field.cpp)
# target_link_libraries(test_collision_field tfield ugvnav_viz lightviz utilities)

# add_executable(test_true_threat test_true_threat.cpp)
# target_link_libraries(test_true_threat tfield ugvnav_viz lightviz utilities)

add_executable(test_collision_threat test_collision_threat.cpp)
target_link_libraries(test_collision_threat tfield ugvnav_viz utilities)

add_executable(test_static_threat test_static_threat.cpp)
target_link_libraries(test_static_threat tfield ugvnav_viz utilities)

add_executable(test_gaussian_cthreat test_gaussian_cthreat.cpp)
target_link_libraries(test_gaussian_cthreat tfield ugvnav_viz utilities)

add_executable(test_threat_field test_threat_field.cpp)
target_link_libraries(test_threat_field tfield ugvnav_viz utilities)

add_executable(test_threat_field_history test_threat_field_history.cpp)
target_link_libraries(test_threat_field_history tfield ugvnav_viz utilities)

add_executable(test_threat_exposure test_threat_exposure.cpp)
target_link_libraries(test_threat_exposure tfield lattice_planner ugvnav_viz utilities)