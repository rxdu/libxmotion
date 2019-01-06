# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_threat_exposure test_threat_exposure.cpp)
target_link_libraries(test_threat_exposure connectivity navviz utils)

add_executable(test_evaluation test_evaluation.cpp)
target_link_libraries(test_evaluation connectivity navviz utils)

add_executable(test_evaluation2 test_evaluation2.cpp)
target_link_libraries(test_evaluation2 connectivity navviz utils)

add_executable(test_evaluation3 test_evaluation3.cpp)
target_link_libraries(test_evaluation3 connectivity navviz utils)