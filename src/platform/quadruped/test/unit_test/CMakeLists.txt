# add unit test
add_executable(utest_quadruped
    utest_unitree_leg.cpp
    utest_config_loader.cpp)
target_link_libraries(utest_quadruped quadruped gtest gmock gtest_main)

gtest_discover_tests(utest_quadruped)
add_test(NAME platform::quadruped COMMAND utest_quadruped)

