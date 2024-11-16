find_package(Threads REQUIRED)

add_library(sensor_imu STATIC
        src/ch_serial.c
        src/imu_hipnuc.cpp)
target_link_libraries(sensor_imu PUBLIC interface async_port Threads::Threads)
target_compile_definitions(sensor_imu PUBLIC ASIO_ENABLE_OLD_SERVICES)
target_compile_options(sensor_imu PUBLIC -fPIC)
target_include_directories(sensor_imu PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE src)

if (BUILD_TESTS)
    add_subdirectory(test)
endif ()

## installation configuration
install(TARGETS sensor_imu
        EXPORT xmotionTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
