# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_motion_model test_motion_model.cpp)
target_link_libraries(test_motion_model cthreat trafficmap lightviz utilities)

add_executable(test_threat_ranking test_threat_ranking.cpp)
target_link_libraries(test_threat_ranking cthreat trafficmap roadmap tfield ugvnav_viz lightviz utilities)

add_executable(test_collision_region test_collision_region.cpp)
target_link_libraries(test_collision_region cthreat trafficmap roadmap tfield ugvnav_viz lightviz utilities)

add_executable(test_collision_threat test_collision_threat.cpp)
target_link_libraries(test_collision_threat cthreat trafficmap roadmap tfield ugvnav_viz lightviz utilities)