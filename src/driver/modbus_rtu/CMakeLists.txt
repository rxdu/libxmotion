find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMODBUS QUIET IMPORTED_TARGET libmodbus)

if (LIBMODBUS_FOUND)
    add_library(modbus_rtu STATIC
        src/modbus_rtu_port.cpp)
    target_link_libraries(modbus_rtu PUBLIC interface logging PkgConfig::LIBMODBUS)
    target_include_directories(modbus_rtu PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
            PRIVATE src)

    #if (BUILD_TESTS)
    #    add_subdirectory(test)
    #endif ()

    ## installation configuration
    install(TARGETS modbus_rtu
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