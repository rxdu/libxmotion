# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_threat_exposure test_threat_exposure.cpp)
target_link_libraries(test_threat_exposure connectivity ugvnav_viz utilities)

add_executable(test_evaluation test_evaluation.cpp)
target_link_libraries(test_evaluation connectivity ugvnav_viz utilities)