add_executable(test_models test_models.cpp)
target_link_libraries(test_models models gtest gtest_main)
gtest_discover_tests(test_models)
