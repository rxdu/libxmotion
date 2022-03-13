add_library(imu STATIC
        # hipnuc imu
        src/ch_serial.c
        src/imu_hipnuc.cpp)
target_link_libraries(imu PUBLIC async_port interface)
target_include_directories(imu PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE src)

if(BUILD_TESTS)
    add_subdirectory(test)
endif()