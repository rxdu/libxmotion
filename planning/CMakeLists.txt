cmake_minimum_required(VERSION 2.8.11)
project(srcl_planning)

## Set directories for 3rd party libraries
# Octomap - required by the octree module
set(OCTOMAP_LIB_BUILT FALSE)
find_path(FOUND_OCTOMAP_PATH liboctomap.so ${CMAKE_SOURCE_DIR}/third_party/octomap/lib/)
find_path(FOUND_OCTOMATH_PATH liboctomath.so ${CMAKE_SOURCE_DIR}/third_party/octomap/lib/)

if(FOUND_OCTOMAP_PATH AND FOUND_OCTOMATH_PATH)
    set(OCTOMAP_LIB_BUILT TRUE)
endif(FOUND_OCTOMAP_PATH AND FOUND_OCTOMATH_PATH)

if(OCTOMAP_LIB_BUILT)
    set(OCTOMAP_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/third_party/octomap/include")
    set(OCTOMAP_LIBRARY_DIRS "${CMAKE_SOURCE_DIR}/third_party/octomap/lib")

    set(OCTOMAP_LIBRARIES
      "${CMAKE_SOURCE_DIR}/third_party/octomap/lib/liboctomap.so"
      "${CMAKE_SOURCE_DIR}/third_party/octomap/lib/liboctomath.so"
    )

    include_directories(${OCTOMAP_INCLUDE_DIRS})

    message(STATUS "Octomap library found")

else(OCTOMAP_LIB_BUILT)
    message(FATAL_ERROR "Please build Octomap library first. Go to /third_party/octomap for build instructions.")
endif(OCTOMAP_LIB_BUILT)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

## Add sub source directories
add_subdirectory(src/quadtree)
add_subdirectory(src/octree)
add_subdirectory(src/square_grid)
add_subdirectory(src/graph)
add_subdirectory(src/map)
add_subdirectory(src/visualizer)
add_subdirectory(src/apps)
add_subdirectory(src/gui)
