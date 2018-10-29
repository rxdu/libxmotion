# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_mc_sim test_mc_sim.cpp)
target_link_libraries(test_mc_sim reachability utilities)

add_executable(test_markov_occupancy test_markov_occupancy.cpp)
target_link_libraries(test_markov_occupancy reachability utilities)