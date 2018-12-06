# Dependency libraries
# find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(CAV_MOTION_LIB_SRC
  # lattice planner
#   src/lattice_graph.cpp
#   src/reference_trajectory.cpp
  # src/lattice_planner.cpp
  # threat evaluation
#   src/lookahead_zone.cpp
    src/cav_motion_manager.cpp
)
add_library(cav_motion STATIC ${CAV_MOTION_LIB_SRC})
target_link_libraries(cav_motion common cav_common local_planner datalink utils)
target_include_directories(cav_motion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

if(BUILD_TESTS)
  add_subdirectory(tests)
endif()
