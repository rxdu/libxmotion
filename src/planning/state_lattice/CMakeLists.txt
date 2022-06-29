# Dependency libraries
# find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(LATTICE_LIB_SRC
  src/point_kinematics.cpp
  src/motion_primitive.cpp
  src/lookup_table.cpp
  src/primitive_generator.cpp
  src/state_lattice.cpp
  # drawing
  src/lattice_draw.cpp
  src/lattice_viz.cpp 
)
add_library(lattice STATIC ${LATTICE_LIB_SRC})
target_link_libraries(lattice control logging utilities cvdraw)
target_include_directories(lattice PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

add_subdirectory(apps)

# if(BUILD_TESTS)
#   add_subdirectory(tests)
# endif()
