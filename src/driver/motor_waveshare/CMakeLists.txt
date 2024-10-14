
add_library(motor_waveshare STATIC
    src/ddsm_210.cpp)
target_link_libraries(motor_waveshare PUBLIC interface logging async_port)
if (AKELC_WITH_MODBUS)
  target_compile_definitions(motor_waveshare PRIVATE AKELC_WITH_MODBUS)
endif ()
target_include_directories(motor_waveshare PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

if (BUILD_APPS)
  add_subdirectory(app)
endif ()

if (BUILD_TESTS)
  add_subdirectory(test)
endif ()

## installation configuration
install(TARGETS motor_waveshare
    EXPORT xmotionTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})