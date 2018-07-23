# Dependency libraries
# find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(GEOMETRY_LIB_SRC
  src/square_grid.cpp
  #src/quad_tree.cpp
  src/cube_array.cpp
)
add_library(geometry STATIC ${GEOMETRY_LIB_SRC})
target_link_libraries(geometry common)
target_include_directories(geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

add_subdirectory(tests)

