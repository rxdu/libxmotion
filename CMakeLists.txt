cmake_minimum_required(VERSION 3.10.2)
project(xmotion VERSION 0.1.0)

message(STATUS "------------------------------------------------")
if (PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME)
  set(BUILD_AS_MODULE OFF)
  message(STATUS "Build libxmotion as a standalone project")
else ()
  set(BUILD_AS_MODULE ON)
  message(STATUS "Build libxmotion as a module")
endif ()
message(STATUS "------------------------------------------------")

find_program(CCACHE_PROGRAM ccache)
if (CCACHE_PROGRAM)
  message(STATUS "Found ccache")
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif ()

## Project Options
option(BUILD_TESTING "Build tests" OFF)
option(FORCE_BUILD_TESTS "Force build tests" OFF)
option(ENABLE_LOGGING "Enable logging" ON)
option(STATIC_CHECK "Enable static check" OFF)

option(ENABLE_TBOT_APP "Enable tbot app" OFF)
option(ENABLE_QUADRUPED_APP "Enable quadruped app" OFF)
option(ENABLE_VISUALIZATION "Enable visualization targets" ON)

if (BUILD_AS_MODULE AND (NOT (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")))
  message(STATUS "Disable visualization targets on non-x86_64 when built as module")
  set(ENABLE_VISUALIZATION OFF)
endif ()

if (ENABLE_VISUALIZATION)
  message(STATUS "Enable visualization targets")
endif ()

## Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (STATIC_CHECK)
  find_program(CPPCHECK cppcheck)
  if (CPPCHECK)
    message(STATUS "Found cppcheck")
    set(CMAKE_CXX_CPPCHECK cppcheck;--std=c++11;--enable=all)
  endif ()
endif ()

## Additional cmake module path
set(USER_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${USER_CMAKE_PATH}/modules")
list(APPEND CMAKE_PREFIX_PATH "/usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu/cmake")

## Set compiler to use c++ 17 features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Use GNUInstallDirs to install libraries into correct locations on all platforms.
include(GNUInstallDirs)

## Put all binary files into /bin and libraries into /lib
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})

## Chosse build type
set(default_build_type "Release")

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)

  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
      "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif ()

if (ENABLE_LOGGING)
  message("-- Enable logging with spdlog -- Yes")
  message("-- Default log file storage location: ~/.xmotion/log")
else (ENABLE_LOGGING)
  message("-- Disable logging with spdlog")
endif (ENABLE_LOGGING)

## Enable test if requested
if (((PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME) AND BUILD_TESTING) OR FORCE_BUILD_TESTS)
  enable_testing()
  include(GoogleTest)
  set(BUILD_TESTS ON)
  message(STATUS "Tests will be built")
else ()
  set(BUILD_TESTS OFF)
  message(STATUS "Tests will not be built")
endif ()

## Dependent libraries
add_subdirectory(third_party)

## Add source directories
add_subdirectory(src)

## Installation setup
# Offer the user the choice of overriding the installation directories
set(INSTALL_LIBDIR ${CMAKE_INSTALL_LIBDIR} CACHE PATH "Installation directory for libraries")
set(INSTALL_BINDIR ${CMAKE_INSTALL_BINDIR} CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDEDIR ${CMAKE_INSTALL_INCLUDEDIR} CACHE PATH "Installation directory for header files")
if (WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKEDIR CMake)
else ()
  set(DEF_INSTALL_CMAKEDIR share/cmake/${PROJECT_NAME})
endif ()
set(INSTALL_CMAKEDIR ${DEF_INSTALL_CMAKEDIR} CACHE PATH "Installation directory for CMake files")

# print installation paths
message(STATUS "Project will be installed to ${CMAKE_INSTALL_PREFIX} with 'make install'")
foreach (p LIB BIN INCLUDE CMAKE)
  file(TO_NATIVE_PATH ${CMAKE_INSTALL_PREFIX}/${INSTALL_${p}DIR} _path)
  message(STATUS " - To install ${p} components to ${_path}")
  unset(_path)
endforeach ()

# export target configuration
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    xmotionConfigVersion.cmake
    VERSION "${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${${PROJECT_NAME}_VERSION_PATCH}"
    COMPATIBILITY SameMajorVersion)

install(EXPORT xmotionTargets
    FILE xmotionTargets.cmake
    NAMESPACE xmotion::
    DESTINATION lib/cmake/xmotion)

configure_file(cmake/xmotionConfig.cmake.in xmotionConfig.cmake @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/xmotionConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/xmotionConfigVersion.cmake"
    DESTINATION lib/cmake/xmotion)

# Packaging support
set(CPACK_DEBIAN_PACKAGE_NAME "libxmotion")
set(CPACK_PACKAGE_VENDOR "Ruixiang Du")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "xMotion Library - a library for motion planning and control")
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/xmotion")
set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
# set(CPACK_DEBIAN_PACKAGE_DEPENDS "abc (>= 0.1.2), xyz (<< 0.3.4)")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libgl1-mesa-dev, libglfw3-dev, libcairo2-dev,
    libtbb-dev, libasio-dev, libboost-all-dev, libgsl-dev, libeigen3-dev, libtbb-dev,
    libopencv-dev, libyaml-cpp-dev, libncurses-dev, libevdev-dev, libmodbus-dev, libpcl-dev")
set(CPACK_GENERATOR "DEB")
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Ruixiang Du (ruixiang.du@gmail.com)")
set(CPACK_SOURCE_IGNORE_FILES .git dist .*build.* /\\\\.DS_Store)
include(CPack)
