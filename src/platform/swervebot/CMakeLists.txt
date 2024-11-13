## Dependency libraries
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

## Add libraries
add_library(swervebot
    # common
    src/ws_sbot_base.cpp)
target_link_libraries(swervebot PUBLIC
    interface
    motor_waveshare
    robot_base
    fsm
    input_hid
    Eigen3::Eigen
    yaml-cpp
    PRIVATE logging)
target_include_directories(swervebot PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

if (BUILD_TESTS)
  add_subdirectory(test)
endif ()

add_subdirectory(app)

## installation configuration
install(TARGETS swervebot
    EXPORT xmotionTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
