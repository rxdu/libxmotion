find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(Libevent REQUIRED IMPORTED_TARGET libevent)

add_library(input_hid STATIC
    src/joystick.cpp
    src/keyboard.cpp
    src/hid_event_listener.cpp
    src/keyboard_handler.cpp
    src/keyboard_mapping.cpp
)
target_link_libraries(input_hid PUBLIC interface logging Threads::Threads PkgConfig::Libevent)
target_compile_definitions(input_hid PUBLIC ASIO_ENABLE_OLD_SERVICES)
target_compile_options(input_hid PUBLIC -fPIC)
target_include_directories(input_hid PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

if (BUILD_TESTS)
  add_subdirectory(test)
endif ()

## installation configuration
install(TARGETS input_hid
    EXPORT xmotionTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
