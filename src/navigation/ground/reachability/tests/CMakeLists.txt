# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_mc_sim test_mc_sim.cpp)
target_link_libraries(test_mc_sim reachability utilities)

add_executable(test_tstate_space test_tstate_space.cpp)
target_link_libraries(test_tstate_space reachability utilities ugvnav_viz)

add_executable(test_tstate_sim test_tstate_sim.cpp)
target_link_libraries(test_tstate_sim reachability utilities ugvnav_viz)

add_executable(test_markov_occupancy test_markov_occupancy.cpp)
target_link_libraries(test_markov_occupancy reachability utilities)

add_executable(test_matrix test_matrix.cpp)
# target_link_libraries(test_matrix)