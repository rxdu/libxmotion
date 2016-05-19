cmake_minimum_required(VERSION 2.8.11)
project(srcl_planning)

## Set compiler to use c++ 11 features
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++14 or C++11 support. Please use a different C++ compiler.")
endif()

## Put all binary files into /bin and libraries into /lib
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

## Set directories for 3rd party libraries
# Octomap - required by the octree module
set(OCTOMAP_LIB_BUILT FALSE)
find_path(FOUND_OCTOMAP_PATH liboctomap.so ${CMAKE_SOURCE_DIR}/src/third_party/octomap/lib/)
find_path(FOUND_OCTOMATH_PATH liboctomath.so ${CMAKE_SOURCE_DIR}/src/third_party/octomap/lib/)

if(FOUND_OCTOMAP_PATH AND FOUND_OCTOMATH_PATH)
    set(OCTOMAP_LIB_BUILT TRUE)
endif(FOUND_OCTOMAP_PATH AND FOUND_OCTOMATH_PATH)

if(OCTOMAP_LIB_BUILT)
    set(OCTOMAP_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/src/third_party/octomap/include")
    set(OCTOMAP_LIBRARY_DIRS "${CMAKE_SOURCE_DIR}/src/third_party/octomap/lib")
     
    set(OCTOMAP_LIBRARIES 
      "${CMAKE_SOURCE_DIR}/src/third_party/octomap/lib/liboctomap.so"
      "${CMAKE_SOURCE_DIR}/src/third_party/octomap/lib/liboctomath.so"
    )

    include_directories(${OCTOMAP_INCLUDE_DIRS})
    
    message(STATUS "Octomap library found")
    
else(OCTOMAP_LIB_BUILT)    
    message(FATAL_ERROR "Please build Octomap library first. Go to /src/third_party/octomap for build instructions.")
endif(OCTOMAP_LIB_BUILT)

## Add sub source directories
add_subdirectory(src/quadtree)
add_subdirectory(src/octree)
add_subdirectory(src/square_grid)
add_subdirectory(src/graph)
add_subdirectory(src/map)
add_subdirectory(src/visualizer)
add_subdirectory(src/apps)
add_subdirectory(src/gui)
