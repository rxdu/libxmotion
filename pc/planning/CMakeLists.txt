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
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

## Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

# Octomap include directories
include_directories(${CMAKE_SOURCE_DIR}/third_party/octomap-1.8.0/octomap/include)
#include_directories(${CMAKE_SOURCE_DIR}/third_party/octomap-1.8.0/octovis/include)
#include_directories(${CMAKE_SOURCE_DIR}/third_party/octomap-1.8.0/dynamicEDT3D/include)
set(Octomap_LIBS octomap octomath)

## Find LCM
#find_package(PkgConfig REQUIRED)
#pkg_search_module(LCM REQUIRED lcm)
find_package(LCM REQUIRED)

if(LCM_NOT_FOUND)
  message(STATUS "LCM is not found. Inter-process communication depends on LCM.")
endif(LCM_NOT_FOUND)

# Include files for LCM
set(LCM_GENERATED_HEADER_PATH ${CMAKE_BINARY_DIR}/include)
include_directories(${LCM_GENERATED_HEADER_PATH})

## Conditional add sources
find_package(GUROBI)
if (NOT GUROBI_FOUND)
  message("Gurobi not found. If you want to use Gurobi, set the GUROBI_HOME variable")
endif()

message("Gurobi found")
include_directories(${GUROBI_INCLUDE_DIRS})

## Add other sub source directories
add_subdirectory(src/geometry)
add_subdirectory(src/polyopt)
add_subdirectory(src/path_repair)
#add_subdirectory(src/graph)
add_subdirectory(src/map)
add_subdirectory(src/local)
add_subdirectory(src/vis)
add_subdirectory(src/examples)
add_subdirectory(src/quad)
add_subdirectory(src/apps)
