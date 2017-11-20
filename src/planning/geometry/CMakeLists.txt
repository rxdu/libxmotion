# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
# find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(GEOMETRY_LIB_SRC
  # data structures
  src/square_grid.cpp
  src/quad_tree.cpp
  src/cube_array.cpp
  # builders
  src/qtree_builder.cpp
  src/sgrid_builder.cpp
  src/sgrid_builder_v2.cpp
  src/cube_array_builder.cpp
  src/graph_builder.cpp
)
add_library(geometry STATIC ${GEOMETRY_LIB_SRC})
target_link_libraries(geometry map ${Octomap_LIBS})
target_include_directories(geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

# Add executables
#add_executable(octomap_server_node octomap_server_node.cpp)
#target_link_libraries(octomap_server_node octomap_utils ${Octomap_LIBS} ${LCM_LIBRARIES})
