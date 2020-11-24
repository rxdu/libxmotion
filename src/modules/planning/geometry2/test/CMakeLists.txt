# add unit test
add_executable(utest_geometry 
    utest_polyline.cpp
    utest_cubicspline.cpp)
target_link_libraries(utest_geometry geometry2 gtest gmock gtest_main)
 
gtest_discover_tests(utest_geometry)
add_test(NAME planning::geometry COMMAND utest_geometry)
