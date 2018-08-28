# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_motion_model test_motion_model.cpp)
target_link_libraries(test_motion_model threatrank trafficmap lightviz utilities)

add_executable(test_threat_ranking test_threat_ranking.cpp)
target_link_libraries(test_threat_ranking threatrank trafficmap roadmap tfield trafficviz lightviz utilities)

