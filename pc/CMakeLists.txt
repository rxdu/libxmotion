cmake_minimum_required(VERSION 2.8.11)
project(srcl_ctrl)

# cmake module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules")

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME srcl_ctrl)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

## Optionally built modules: on/off
set(ENABLE_LOGGING on)
set(BUILD_TESTS off)

if(ENABLE_LOGGING)
	message( "-- Enable logging with spdlog -- Yes" )
	message( "-- Deafult log directory: srcl_ctrl/control/log " )
else(ENABLE_LOGGING)
	message( "-- Enable logging with spdlog -- No" )
endif(ENABLE_LOGGING)

## Config common include directories
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/third_party/)

## Add other sub source directories
#add_subdirectory(utils)
add_subdirectory(common)
add_subdirectory(control)
#add_subdirectory(planning)
add_subdirectory(third_party)

## Add test source directories
if(BUILD_TESTS)
    add_subdirectory(gtests)
endif()
