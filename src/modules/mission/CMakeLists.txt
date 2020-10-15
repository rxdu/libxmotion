# Dependency libraries
# find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(MISSION_LIB_SRC
    # state
    src/vehicle_state.cpp
    # controller
    src/virtual_controller.cpp
    # planner
    src/reference_route.cpp
    src/route_planner.cpp
    # motion manager
    src/motion_manager.cpp
    # mission control
    src/mission_control.cpp
)
add_library(mission STATIC ${MISSION_LIB_SRC})
target_link_libraries(mission navtypes datalink traffic local_planner utils)
target_include_directories(mission PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

add_subdirectory(apps)

if(BUILD_TESTS)
  add_subdirectory(tests)
endif()
