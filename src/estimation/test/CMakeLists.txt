add_executable(test_mekf6 test_mekf6.cpp)
target_link_libraries(test_mekf6 estimation gtest gtest_main)
gtest_discover_tests(test_mekf6)

add_executable(test_mekf9 test_mekf9.cpp)
target_link_libraries(test_mekf9 estimation gtest gtest_main)
gtest_discover_tests(test_mekf9)
