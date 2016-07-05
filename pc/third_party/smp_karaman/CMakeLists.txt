cmake_minimum_required(VERSION 2.8.11)
project(smp_karaman_dep_lib)

# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/halton)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/inc_mu_mc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/kdtree)

# Dependency libraries
#find_package(OpenCV REQUIRED)

## Add sub source directories
add_subdirectory(smp/external_libraries)
