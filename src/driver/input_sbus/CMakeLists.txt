find_package(Threads REQUIRED)

add_library(input_sbus STATIC
    src/sbus_decoder.cpp
    src/sbus_receiver.cpp
    src/sbus_receiver_impl.cpp)
target_link_libraries(input_sbus PUBLIC interface rpi_sbus)
target_include_directories(input_sbus PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

if (BUILD_TESTS)
  add_subdirectory(test)
endif ()

## installation configuration
install(TARGETS input_sbus
    EXPORT xmotionTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
