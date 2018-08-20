# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_motion_model test_motion_model.cpp)
target_link_libraries(test_motion_model rank lightviz utilities)

add_executable(test_threat_ranking test_threat_ranking.cpp)
target_link_libraries(test_threat_ranking rank roadmap tfield vehviz lightviz utilities)