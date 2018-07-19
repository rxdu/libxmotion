# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_dynamics test_dynamics.cpp)
target_link_libraries(test_dynamics reachability)

add_executable(test_propagator test_propagator.cpp)
target_link_libraries(test_propagator reachability)

add_executable(test_mc_sim test_mc_sim.cpp)
target_link_libraries(test_mc_sim reachability utilities)

add_executable(test_bigaussian test_bigaussian.cpp)
target_link_libraries(test_bigaussian reachability utilities)