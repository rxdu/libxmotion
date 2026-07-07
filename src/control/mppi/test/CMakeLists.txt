add_executable(test_mppi_core test_mppi_core.cpp)
target_link_libraries(test_mppi_core mppi gtest gtest_main)
gtest_discover_tests(test_mppi_core)

add_executable(test_mppi_control test_mppi_control.cpp)
target_link_libraries(test_mppi_control mppi gtest gtest_main)
gtest_discover_tests(test_mppi_control)

add_executable(test_mppi_samplers test_mppi_samplers.cpp)
target_link_libraries(test_mppi_samplers mppi gtest gtest_main)
gtest_discover_tests(test_mppi_samplers)

add_executable(test_mppi_quadruped test_mppi_quadruped.cpp)
target_link_libraries(test_mppi_quadruped mppi gtest gtest_main)
gtest_discover_tests(test_mppi_quadruped)

add_executable(test_mppi_introspection test_mppi_introspection.cpp)
target_link_libraries(test_mppi_introspection mppi gtest gtest_main)
gtest_discover_tests(test_mppi_introspection)

if (XMNAV_WITH_CUDA)
  add_executable(test_mppi_cuda test_mppi_cuda.cpp)
  target_link_libraries(test_mppi_cuda mppi_cuda gtest gtest_main)
  gtest_discover_tests(test_mppi_cuda)
endif ()
