# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(LCM REQUIRED lcm)

# Add libraries
set(MAP3D_LIB_SRC
    octomap_server.cpp
)
add_library(octomap_utils STATIC ${MAP3D_LIB_SRC})
target_link_libraries(octomap_utils ${Octomap_LIBS})

# Add executables
add_executable(octree_example octomap_example.cpp)
target_link_libraries(octree_example ${Octomap_LIBS})

add_executable(octomap_server_node octomap_server_node.cpp)
target_link_libraries(octomap_server_node octomap_utils ${Octomap_LIBS} ${LCM_LIBRARIES})
