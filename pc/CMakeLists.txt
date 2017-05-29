cmake_minimum_required(VERSION 2.8.11)
project(srcl_ctrl)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME srcl_ctrl)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

## Put all binary files into /bin and libraries into /lib
#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
#set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
#set(ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

## Choose to enable g3log or not: on/off
set(ENABLE_LOGGING off)

if(ENABLE_LOGGING)
	message( "-- Enable logging with g3log -- Yes" )
	message( "-- Deafult log directory: srcl_ctrl/control/log " )
	include_directories(${CMAKE_SOURCE_DIR}/third_party/g3log/src)
else(ENABLE_LOGGING)
	message( "-- Enable logging with g3log -- No" )
endif(ENABLE_LOGGING)

set(BUILD_TESTS off)

## Config common include directories
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/planning/src)
include_directories(${CMAKE_SOURCE_DIR}/third_party/)

## Config octomap include directories
include_directories(${CMAKE_SOURCE_DIR}/third_party/octomap-1.8.0/octomap/include)
include_directories(${CMAKE_SOURCE_DIR}/third_party/octomap-1.8.0/octovis/include)
include_directories(${CMAKE_SOURCE_DIR}/third_party/octomap-1.8.0/dynamicEDT3D/include)
set(Octomap_LIBS octomap octomath)

## Config ompl include directories
include_directories(${CMAKE_SOURCE_DIR}/third_party/ompl-1.2.1/src)

## Other include directories
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/control/src)
#include_directories(${CMAKE_SOURCE_DIR}/planning/src)
#include_directories(${CMAKE_SOURCE_DIR}/utils/src)

## Find LCM
find_package(PkgConfig REQUIRED)
pkg_search_module(LCM REQUIRED lcm)

if(LCM_NOT_FOUND)
  message(STATUS "LCM is not found. Inter-process communication depends on LCM.")
endif(LCM_NOT_FOUND)

set(LCM_GENERATED_HEADER_PATH ${CMAKE_BINARY_DIR}/include)
include_directories(${LCM_GENERATED_HEADER_PATH})
# Include files for LCM
include_directories(${CMAKE_BINARY_DIR}/include)

## Add test source directories
if(BUILD_TESTS)
    add_subdirectory(gtests)
endif()

## Add other sub source directories
add_subdirectory(utils)
set(LCM_GENERATED_HEADER_PATH ${CMAKE_BINARY_DIR}/include)
include_directories(${LCM_GENERATED_HEADER_PATH})
# Include files for LCM
	include_directories(${CMAKE_BINARY_DIR}/include)
# add comm and control module
add_subdirectory(control)
add_subdirectory(planning)
add_subdirectory(third_party)
