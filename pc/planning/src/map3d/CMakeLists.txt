# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(LCM REQUIRED lcm)

# Add libraries
set(MAP3D_LIB_SRC
    octomap_server.cpp
    transformation.cpp
)
add_library(map3d STATIC ${MAP3D_LIB_SRC})
target_link_libraries(map3d map geometry ${Octomap_LIBS} ${LCM_LIBRARIES})

# Add executables
add_executable(octree_example tests/octomap_example.cpp)
target_link_libraries(octree_example ${Octomap_LIBS})

#add_executable(test_octomap tests/test_octomap.cpp)
#target_link_libraries(test_octomap local3d ${Octomap_LIBS})

#add_executable(test_cubebuilder tests/test_cubebuilder.cpp)
#target_link_libraries(test_cubebuilder map geometry local3d ${Octomap_LIBS} ${LCM_LIBRARIES})
