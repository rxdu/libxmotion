# add unit test
add_executable(utest_motor_vesc
        utest_vesc_packet.cpp)
target_link_libraries(utest_motor_vesc motor_vesc gtest gmock gtest_main)

gtest_discover_tests(utest_motor_vesc)
add_test(NAME driver::motor_vesc COMMAND utest_motor_vesc)

add_subdirectory(devel)
