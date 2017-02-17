# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(OpenCV REQUIRED)

# Add libraries
set(GEOMETRY_LIB_SRC
  # data structures
  square_grid/square_grid.cpp
  quadtree/quad_tree.cpp
  cube_array/cube_array.cpp
  # builders
  qtree_builder.cpp
  sgrid_builder.cpp
  sgrid_builder_v2.cpp
  cube_array_builder.cpp
  graph_builder.cpp
)
add_library(geometry STATIC ${GEOMETRY_LIB_SRC})
target_link_libraries(geometry map ${OpenCV_LIBS} ${Octomap_LIBS})

# Add executables
#add_executable(octomap_server_node octomap_server_node.cpp)
#target_link_libraries(octomap_server_node octomap_utils ${Octomap_LIBS} ${LCM_LIBRARIES})
