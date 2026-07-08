add_executable(test_pid test_pid.cpp)
target_link_libraries(test_pid pid gtest gtest_main)
gtest_discover_tests(test_pid)
