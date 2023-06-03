find_package(Threads REQUIRED)

add_library(motor_vesc STATIC
        src/vesc_can_interface.cpp
        src/vesc_status_packet.cpp
        src/vesc_cmd_packet.cpp)
target_link_libraries(motor_vesc PUBLIC interface async_port Threads::Threads)
target_compile_definitions(motor_vesc PUBLIC ASIO_ENABLE_OLD_SERVICES)
target_compile_options(motor_vesc PUBLIC -fPIC)
target_include_directories(motor_vesc PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE src)

#if (BUILD_TESTS)
#    add_subdirectory(test)
#endif ()

## installation configuration
install(TARGETS motor_vesc
        EXPORT roboswTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
