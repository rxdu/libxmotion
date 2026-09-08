add_executable(utest_eigen_io utest_eigen_io.cpp)
target_link_libraries(utest_eigen_io map_processing stb gtest gmock gtest_main)
target_include_directories(utest_eigen_io PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)
# This binary links PCL and OpenCV. Where those ship as many small shared
# libraries (Homebrew pulls in ~158), dyld needs far longer to start the
# process than the 5 s default allows -- discovery then times out and CMake
# deletes the executable, surfacing as utest_eigen_io_NOT_BUILT. Defer
# discovery to test time and give it room; both are no-ops where startup is
# already fast.
gtest_discover_tests(utest_eigen_io
    DISCOVERY_MODE PRE_TEST
    DISCOVERY_TIMEOUT 120)

#if (ENABLE_VISUALIZATION)
#  add_executable(test_pgm_map test_pgm_map.cpp)
#  target_link_libraries(test_pgm_map map_processing)
#
#  add_executable(test_pc_processor test_pc_processor.cpp)
#  target_link_libraries(test_pc_processor map_processing)
#
#  add_executable(test_traj_processor test_traj_processor.cpp)
#  target_link_libraries(test_traj_processor map_processing)
#
#  add_executable(test_occupancy_map test_occupancy_map.cpp)
#  target_link_libraries(test_occupancy_map map_processing)
#endif ()