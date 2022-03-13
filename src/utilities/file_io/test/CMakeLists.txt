# add unit test
add_executable(utest_fileio
    utest_fileio.cpp)
target_link_libraries(utest_fileio file_io gtest gmock gtest_main)
 
gtest_discover_tests(utest_fileio)
add_test(NAME utilities::file_io COMMAND utest_fileio)

add_subdirectory(devel)
