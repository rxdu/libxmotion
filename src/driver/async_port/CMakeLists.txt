find_package(Threads REQUIRED)

add_library(async_port STATIC
        src/async_serial.cpp
        src/async_can.cpp)
target_link_libraries(async_port PUBLIC interface Threads::Threads)
target_compile_definitions(async_port PUBLIC ASIO_ENABLE_OLD_SERVICES)
target_compile_options(async_port PUBLIC -fPIC)
target_include_directories(async_port PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE src)

if (BUILD_TESTS)
  add_subdirectory(test)
endif ()

## installation configuration
install(TARGETS async_port
        EXPORT xmotionTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
