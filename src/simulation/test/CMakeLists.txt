add_executable(test_sim_loop test_sim_loop.cpp)
target_link_libraries(test_sim_loop sim mppi gtest gtest_main)
gtest_discover_tests(test_sim_loop)
