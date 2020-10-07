# Dependency libraries
# find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(CAV_MOTION_LIB_SRC
    # state
    src/vehicle_state.cpp
    # controller
    src/virtual_controller.cpp
    # planner
    src/reference_route.cpp
    src/route_planner.cpp
    # motion manager
    src/cav_motion_manager.cpp
    # mission control
    src/cav_mission_control.cpp
)
add_library(cav_motion STATIC ${CAV_MOTION_LIB_SRC})
target_link_libraries(cav_motion adtypes datalink traffic local_planner utils)
target_include_directories(cav_motion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

add_subdirectory(apps)

if(BUILD_TESTS)
  add_subdirectory(tests)
endif()
