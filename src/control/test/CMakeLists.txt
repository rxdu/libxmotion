# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
# add_executable(test_bicycle_kin test_bicycle_kin.cpp)
# target_link_libraries(test_bicycle_kin model)

add_executable(test_propagator test_propagator.cpp)
target_link_libraries(test_propagator control)

add_executable(test_propagator2 test_propagator2.cpp)
target_link_libraries(test_propagator2 control)

add_executable(test_pid test_pid.cpp)
target_link_libraries(test_pid control)