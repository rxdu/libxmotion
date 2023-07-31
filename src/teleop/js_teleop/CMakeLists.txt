add_library(js_teleop src/joystick_teleop.cpp)
target_link_libraries(js_teleop PUBLIC input_joystick)
target_include_directories(js_teleop PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE src)

add_subdirectory(app)

#if(BUILD_TESTS)
#    add_subdirectory(test)
#endif()