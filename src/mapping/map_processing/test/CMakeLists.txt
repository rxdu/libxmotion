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