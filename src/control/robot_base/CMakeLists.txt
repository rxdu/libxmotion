## Dependency libraries
find_package(Eigen3 REQUIRED NO_MODULE)

## Add libraries
add_library(robot_base
    src/speed_actuator_group.cpp
    src/differential_drive_robot.cpp)
target_link_libraries(robot_base PUBLIC interface Eigen3::Eigen)
target_include_directories(robot_base PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

if (BUILD_TESTS)
  add_subdirectory(test)
endif ()

if (BUILD_APPS)
  add_subdirectory(app)
endif ()

## installation configuration
install(TARGETS robot_base
    EXPORT xmotionTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
