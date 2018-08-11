# Dependency libraries
# find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(LATTICE_PLANNER_LIB_SRC
  src/lattice_graph.cpp
  src/lattice_planner.cpp
)
add_library(lattice_planner STATIC ${LATTICE_PLANNER_LIB_SRC})
target_link_libraries(lattice_planner lattice graph roadmap reachability utilities)
target_compile_options(lattice_planner PUBLIC "-frounding-math")
target_include_directories(lattice_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

if(BUILD_TESTS)
  add_subdirectory(tests)
endif()
