if (TARGET async_port)
  add_executable(test_sbus test_sbus.cpp)
  target_link_libraries(test_sbus PRIVATE async_port input_sbus)
endif ()

if (TARGET rpi_sbus)
  add_executable(test_blocking_receiver test_blocking_receiver.cpp)
  target_link_libraries(test_blocking_receiver PRIVATE rpi_sbus)

  add_executable(test_sbus_receiver test_sbus_receiver.cpp)
  target_link_libraries(test_sbus_receiver PRIVATE rpi_sbus input_sbus)
endif ()