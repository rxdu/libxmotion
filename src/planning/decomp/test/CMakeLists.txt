# add unit test
add_executable(utest_decomp
    utest_rect_grid_base.cpp
    utest_square_grid.cpp)
target_link_libraries(utest_decomp PRIVATE decomp gtest gmock gtest_main)
 
gtest_discover_tests(utest_decomp)
add_test(NAME planning::decomp COMMAND utest_decomp)

add_subdirectory(devel)
