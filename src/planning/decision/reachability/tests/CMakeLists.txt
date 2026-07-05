# Standalone development executables (not ctest-registered: they run
# simulations and write output files; run manually during development).

add_executable(test_mc_sim test_mc_sim.cpp)
target_link_libraries(test_mc_sim reachability)

add_executable(test_bigaussian test_bigaussian.cpp)
target_link_libraries(test_bigaussian reachability)

add_executable(test_matrix test_matrix.cpp)
target_link_libraries(test_matrix Eigen3::Eigen)

# Excluded until their dependencies are revived:
# - test_tstate_space, test_tstate_sim -> trafficviz (retired)
# - test_markov_occupancy, test_markov_command, test_markov_command2,
#   test_markov_motion -> file_io/matrix_file (retired)
