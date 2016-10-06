cmake_minimum_required(VERSION 2.8.11)
project(srcl_ctrl)

## Put all binary files into /bin and libraries into /lib
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

## Choose to enable g3log or not: on/off
set(ENABLE_G3LOG on)

if(ENABLE_G3LOG)
	message( "-- Enable logging with g3log -- Yes" )
	message( "-- Deafult log directory: srcl_ctrl/control/log " )
	include_directories(${CMAKE_SOURCE_DIR}/third_party/g3log_srcl/src)
else(ENABLE_G3LOG)
	message( "-- Enable logging with g3log -- No" )
endif(ENABLE_G3LOG)

set(BUILD_TESTS on)

## Config eigen include directories
include_directories(${CMAKE_SOURCE_DIR}/third_party/)

## Config octomap include directories
include_directories(${CMAKE_SOURCE_DIR}/third_party/octomap-1.8.0/octomap/include)
include_directories(${CMAKE_SOURCE_DIR}/third_party/octomap-1.8.0/octovis/include)
include_directories(${CMAKE_SOURCE_DIR}/third_party/octomap-1.8.0/dynamicEDT3D/include)
set(Octomap_LIBS octomap octomath)

## Config ompl include directories
include_directories(${CMAKE_SOURCE_DIR}/third_party/ompl-1.2.1/src)

## Config smp include directories
#include_directories(${CMAKE_SOURCE_DIR}/third_party/smp_karaman)
# SMP is header only, but it depends on 3rd party libraries that needs to be linked against
#set(SMP_KARAMAN_LIB smp_karaman_dep_lib)

## Other include directories
include_directories(${CMAKE_SOURCE_DIR})

## Conditionally add source directories
## Check dependencies for Comm and Control/Planning module
find_package(PkgConfig REQUIRED)
pkg_search_module(LCM REQUIRED lcm)

if(LCM_FOUND)
	set(LCM_GENERATED_HEADER_PATH ${CMAKE_BINARY_DIR}/include)
	include_directories(${LCM_GENERATED_HEADER_PATH})	
	# Include files for LCM
    include_directories(${CMAKE_BINARY_DIR}/include)
	# add comm and control module
	add_subdirectory(comm)            # <-----------------------------------	
	add_subdirectory(control)         # <-----------------------------------	
elseif(LCM_FOUND)
    message(STATUS "LCM is not found. IPC functions will not work without LCM.")
	message(STATUS "Key components missing. The comm, control modules and part of planning module will not be built.")
endif(LCM_FOUND)

## Check dependencies for GUI module
find_package(VTK 7.0)

if (VTK_FOUND)
	add_subdirectory(visualization)    # <-----------------------------------
elseif(VTK_FOUND)
	message(STATUS "GUI module will not be built because not all dependent libraries can be found.")
endif(VTK_FOUND)

## Add other sub source directories
add_subdirectory(utils)   
add_subdirectory(planning)
add_subdirectory(third_party)        
