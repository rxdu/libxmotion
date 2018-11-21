cmake_minimum_required(VERSION 3.0.0)
project(librav)

# cmake module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules")

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

## Chosse build type
set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)

## Optionally built modules: ON/OFF
set(ENABLE_LOGGING ON)
set(BUILD_TESTS ON)

if(ENABLE_LOGGING)
	message( "-- Enable logging with spdlog -- Yes" )
	message( "-- Default log file storage location: librav/data/log" )
else(ENABLE_LOGGING)
	message( "-- Enable logging with spdlog -- No" )
endif(ENABLE_LOGGING)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Add sub source directories
# Core packages of librav
add_subdirectory(kernel)
add_subdirectory(third_party)

# Packages built on top of core
add_subdirectory(mcontrol)
add_subdirectory(navigation)
add_subdirectory(platforms)

# # Publication-related code
add_subdirectory(publication)

## Add test source directories
if(BUILD_TESTS)
    add_subdirectory(unit_tests) 
endif()
