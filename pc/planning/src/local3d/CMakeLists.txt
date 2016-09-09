# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(LCM REQUIRED lcm)

# Add libraries
set(LOCAL3DMAP_LIB_SRC
    octomap_server.cpp
    cube_array_builder.cpp
)
add_library(octomap_utils STATIC ${LOCAL3DMAP_LIB_SRC})
target_link_libraries(octomap_utils map cubearray ${Octomap_LIBS} ${LCM_LIBRARIES})

# Add executables
add_executable(octree_example octomap_example.cpp)
target_link_libraries(octree_example ${Octomap_LIBS})

add_executable(octomap_server_node octomap_server_node.cpp)
target_link_libraries(octomap_server_node octomap_utils ${Octomap_LIBS} ${LCM_LIBRARIES})

add_executable(test_octomap test_octomap.cpp)
target_link_libraries(test_octomap octomap_utils ${Octomap_LIBS})

add_executable(test_cubebuilder test_cubebuilder.cpp)
target_link_libraries(test_cubebuilder map octomap_utils ${Octomap_LIBS} ${LCM_LIBRARIES})