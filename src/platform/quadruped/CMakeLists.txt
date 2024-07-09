## Dependency libraries
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)
find_package(unitree_sdk2 REQUIRED)

## Add libraries
add_library(quadruped
    # common
    src/config_loader.cpp
    # event handler
    src/event_handler/hid_event_handler.cpp
    # model
    src/robot_model/unitree_motor.cpp
    src/robot_model/unitree_leg.cpp
    src/robot_model/unitree_dog.cpp
    src/robot_model/unitree_model_profile.cpp
    # controllers
    src/control_mode_fsm.cpp
    src/control_mode/fixed_stand_mode.cpp
    src/control_mode/free_stand_mode.cpp
    src/control_mode/move_base_mode.cpp
    src/control_mode/passive_mode.cpp
    src/control_mode/trotting_mode.cpp
    # system
    src/quadruped_system.cpp)
target_link_libraries(quadruped PUBLIC
    interface
    fsm time
    input_hid
    unitree_sdk2
    Eigen3::Eigen
    yaml-cpp
    PRIVATE logging)
target_include_directories(quadruped PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

if (BUILD_TESTS)
  add_subdirectory(test)
endif ()

#if (BUILD_APPS)
add_subdirectory(app)
#endif ()

## installation configuration
install(TARGETS quadruped
    EXPORT xmotionTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
