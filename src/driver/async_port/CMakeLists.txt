find_package(Threads REQUIRED)

add_library(async_port STATIC 
    # interface
    src/async_serial.cpp
    src/async_can.cpp)
target_link_libraries(async_port PUBLIC Threads::Threads)
target_compile_definitions(async_port PUBLIC ASIO_ENABLE_OLD_SERVICES)
target_include_directories(async_port PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# if(BUILD_TESTS)
#    add_subdirectory(tests)
# endif()