if (ENABLE_TARGET_DESKTOP)
  add_executable(test_pgm_map test_pgm_map.cpp)
  target_link_libraries(test_pgm_map map)
endif ()