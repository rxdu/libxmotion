# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(LCM REQUIRED lcm)

# Add libraries
set(MAP_LIB_SRC
  image_utils.cpp
  map_utils.cpp
  qtree_builder.cpp
  sgrid_builder.cpp
  graph_builder.cpp
)
add_library(map STATIC ${MAP_LIB_SRC})
target_link_libraries(map squaregrid qtree cubearray ${OpenCV_LIBS})

# Add executables
#add_executable(octomap_server_node octomap_server_node.cpp)
#target_link_libraries(octomap_server_node octomap_utils ${Octomap_LIBS} ${LCM_LIBRARIES})
