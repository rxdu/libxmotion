# Dependency libraries
# find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(LATTICE_LIB_SRC
  src/lattice_generator.cpp
  src/lattice_manager.cpp
  # src/lattice_planner.cpp
)
add_library(lattice STATIC ${LATTICE_LIB_SRC})
target_link_libraries(lattice graph reachability utilities)
target_include_directories(lattice PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

if(BUILD_TESTS)
  add_subdirectory(tests)
endif()
