cmake_minimum_required(VERSION 3.16.0)
project(auto_drive)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    message(STATUS "Found ccache")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif()

## Project Options
option(BUILD_TESTS ON)
option(ENABLE_LOGGING ON)
option(STATIC_CHECK OFF)
# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)

## Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(STATIC_CHECK)
    find_program(CPPCHECK cppcheck)
    if(CPPCHECK)
        message(STATUS "Found cppcheck")
        set(CMAKE_CXX_CPPCHECK cppcheck;--std=c++11;--enable=all)
    endif() 
endif()

## Additional cmake module path
set(USER_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${USER_CMAKE_PATH}/modules")
list(APPEND CMAKE_PREFIX_PATH "/usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu/cmake")

## Set compiler to use c++ 17 features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Chosse build type
set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

if(ENABLE_LOGGING)
	message( "-- Enable logging with spdlog -- Yes" )
	message( "-- Default log file storage location: librav/data/log" )
else(ENABLE_LOGGING)
	message( "-- Enable logging with spdlog -- No" )
endif(ENABLE_LOGGING)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

## Add source directories
add_subdirectory(src)

## Build tests
if(PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME AND BUILD_TESTS)
    message(STATUS "Tests will be built")
    enable_testing()
    include(GoogleTest)
    # add_subdirectory(tests)
else()
    message(STATUS "Tests will not be built")
endif()

## Use GNUInstallDirs to install libraries into correct locations on all platforms.
include(GNUInstallDirs)
message(STATUS "Project will be installed to ${CMAKE_INSTALL_PREFIX} with 'make install'")

## Put all binary files into /bin and libraries into /lib
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
