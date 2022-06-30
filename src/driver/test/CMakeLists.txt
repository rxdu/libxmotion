# add unit test
add_executable(utest_driver
    utest_vesc_packet.cpp)
target_link_libraries(utest_driver PRIVATE driver gtest gmock gtest_main)
 
gtest_discover_tests(utest_driver)
add_test(NAME driver::utest_driver COMMAND utest_driver)

add_subdirectory(devel)
