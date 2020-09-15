cmake_minimum_required(VERSION 3.0.0)
project(librav)

# cmake module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules")

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

## Set compiler to use c++ 11 features
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Chosse build type
set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)

## Optionally built modules: ON/OFF
set(ENABLE_LOGGING ON)
set(BUILD_TESTS OFF)

if(ENABLE_LOGGING)
	message( "-- Enable logging with spdlog -- Yes" )
	message( "-- Default log file storage location: librav/data/log" )
else(ENABLE_LOGGING)
	message( "-- Enable logging with spdlog -- No" )
endif(ENABLE_LOGGING)

## Add source directories
add_subdirectory(src)

## Add test source directories
if(BUILD_TESTS)
    add_subdirectory(unit_tests) 
endif()
