# Dependency libraries
# find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(LATTICE_PLANNER_LIB_SRC
  src/lattice_graph.cpp
  src/lattice_trajectory.cpp
  # src/local_planner.cpp
)
add_library(local_planner STATIC ${LATTICE_PLANNER_LIB_SRC})
target_link_libraries(local_planner lattice graph traffic tfield utilities)
target_include_directories(local_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

if(BUILD_TESTS)
  add_subdirectory(tests)
endif()
