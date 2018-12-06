# Dependency libraries
# find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(CAV_PLANNER_LIB_SRC
  # lattice planner
#   src/lattice_graph.cpp
#   src/reference_trajectory.cpp
  # src/lattice_planner.cpp
  # threat evaluation
#   src/lookahead_zone.cpp
    src/cav_planner.cpp
)
add_library(cav_planner STATIC ${CAV_PLANNER_LIB_SRC})
target_link_libraries(cav_planner common local_planner cav_common utils)
target_include_directories(cav_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

if(BUILD_TESTS)
  add_subdirectory(tests)
endif()
