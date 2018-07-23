cmake_minimum_required(VERSION 3.0.0)
project(librav)

# cmake module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules")

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

## Chosse build type
# set(CMAKE_BUILD_TYPE Debug)

## Optionally built modules: on/off
set(ENABLE_LOGGING on)
set(BUILD_TESTS on)

if(ENABLE_LOGGING)
	message( "-- Enable logging with spdlog -- Yes" )
	message( "-- Default log file storage location: librav/data/log" )
else(ENABLE_LOGGING)
	message( "-- Enable logging with spdlog -- No" )
endif(ENABLE_LOGGING)

## Add sub source directories
add_subdirectory(examples)

add_subdirectory(core)
add_subdirectory(platforms)
add_subdirectory(third_party)

add_subdirectory(experiments)

## Add test source directories
if(BUILD_TESTS)
    # add_subdirectory(gtests) # broken
endif()
