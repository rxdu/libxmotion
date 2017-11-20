cmake_minimum_required(VERSION 2.8.11)
project(librav)

# cmake module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules")

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

include(cmake/uavcantypes.cmake)
uavcantypes_build()

## Optionally built modules: on/off
set(ENABLE_LOGGING on)
set(BUILD_TESTS off)

if(ENABLE_LOGGING)
	message( "-- Enable logging with spdlog -- Yes" )
	message( "-- Default log file storage location: librav/data/log" )
else(ENABLE_LOGGING)
	message( "-- Enable logging with spdlog -- No" )
endif(ENABLE_LOGGING)

## Config common include directories
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/libraries/)
include_directories(${CMAKE_SOURCE_DIR}/libraries/octomap-1.8.0/octomap/include)

set(Octomap_LIBS octomap octomath)

## Add other sub source directories
add_subdirectory(examples)
add_subdirectory(vtalk)
add_subdirectory(libraries)
add_subdirectory(common)
add_subdirectory(control)
add_subdirectory(planning)
add_subdirectory(utility)
add_subdirectory(vis)
add_subdirectory(fastplot)
add_subdirectory(platforms/quadrotor)
add_subdirectory(platforms/rc_car)
add_subdirectory(platforms/simulator)

## Add test source directories
if(BUILD_TESTS)
    add_subdirectory(gtests)
endif()
