if (TARGET async_port)
  add_executable(test_sbus test_sbus.cpp)
  target_link_libraries(test_sbus PRIVATE async_port input_sbus)
endif ()