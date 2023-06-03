find_package(Threads REQUIRED)

add_library(input_joystick STATIC
        src/joystick.cpp)
target_link_libraries(input_joystick PUBLIC interface async_port Threads::Threads)
target_compile_definitions(input_joystick PUBLIC ASIO_ENABLE_OLD_SERVICES)
target_compile_options(input_joystick PUBLIC -fPIC)
target_include_directories(input_joystick PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE src)

if (BUILD_TESTS)
    add_subdirectory(test)
endif ()

## installation configuration
install(TARGETS input_joystick
        EXPORT roboswTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
