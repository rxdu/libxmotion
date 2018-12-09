# Dependency libraries
# find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(CAV_MOTION_LIB_SRC
    # controller
    src/cav_virtual_controller.cpp
    # planner
    src/reference_route.cpp
    src/route_planner.cpp
    # motion manager
    src/cav_motion_manager.cpp
)
add_library(cav_motion STATIC ${CAV_MOTION_LIB_SRC})
target_link_libraries(cav_motion common datalink cav_common traffic local_planner utils)
target_include_directories(cav_motion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

if(BUILD_TESTS)
  add_subdirectory(tests)
endif()
