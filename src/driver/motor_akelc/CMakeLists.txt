set(AKELC_WITH_MODBUS OFF)
set(AKELC_WITH_CANOPEN OFF)

if (LIBMODBUS_FOUND)
  set(AKELC_WITH_MODBUS ON)
endif ()

if (AKELC_WITH_MODBUS)
  set(AKELC_MODBUS_IMPL src/motor_akelc_modbus.cpp)
  set(AKELC_MODBUS_LIB modbus_rtu)
endif ()

if (AKELC_WITH_MODBUS OR AKELC_WITH_CANOPEN)
  add_library(motor_akelc STATIC
      src/motor_akelc.cpp
      ${AKELC_MODBUS_IMPL})
  target_link_libraries(motor_akelc PUBLIC interface logging ${AKELC_MODBUS_LIB})
  if (AKELC_WITH_MODBUS)
    target_compile_definitions(motor_akelc PRIVATE AKELC_WITH_MODBUS)
  endif ()
  target_include_directories(motor_akelc PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      PRIVATE src)

  add_subdirectory(app)

  if (BUILD_TESTS)
    add_subdirectory(test)
  endif ()

  ## installation configuration
  install(TARGETS motor_akelc
      EXPORT xmotionTargets
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib
      RUNTIME DESTINATION bin
      INCLUDES DESTINATION include)

  install(DIRECTORY include/
      DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
else ()
  message(FATAL_ERROR "libmodbus not found, please install with: sudo apt-get install libmodbus-dev")
endif ()