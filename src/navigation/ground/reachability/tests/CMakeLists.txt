# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_mc_sim test_mc_sim.cpp)
target_link_libraries(test_mc_sim reachability utils)

add_executable(test_tstate_space test_tstate_space.cpp)
target_link_libraries(test_tstate_space reachability utils navviz)

add_executable(test_tstate_sim test_tstate_sim.cpp)
target_link_libraries(test_tstate_sim reachability utils navviz)

add_executable(test_markov_occupancy test_markov_occupancy.cpp)
target_link_libraries(test_markov_occupancy reachability utils)

add_executable(test_markov_command test_markov_command.cpp)
target_link_libraries(test_markov_command reachability utils)

add_executable(test_markov_command2 test_markov_command2.cpp)
target_link_libraries(test_markov_command2 reachability utils)

add_executable(test_markov_motion test_markov_motion.cpp)
target_link_libraries(test_markov_motion reachability utils)

add_executable(test_bigaussian test_bigaussian.cpp)
target_link_libraries(test_bigaussian reachability logging)

add_executable(test_matrix test_matrix.cpp)
# target_link_libraries(test_matrix)