cmake_minimum_required(VERSION 3.10.2)
project(xmNavigation VERSION 0.2.0)

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
option(STATIC_CHECK "Enable static check" OFF)

option(ENABLE_VISUALIZATION "Enable visualization targets" ON)
option(XMOTION_DEV_MODE "Development mode forces building tests" OFF)
option(XMOTION_WERROR "Treat warnings as errors (-Wall -Wextra -Werror)" OFF)
option(XMNAV_WITH_CUDA "Build CUDA rollout backends (requires CUDA toolkit)" OFF)

if (DEFINED ENV{ROS_DISTRO})
  set(BUILD_WITH_ROS ON)
endif ()

if (BUILD_WITH_ROS OR BUILD_AS_MODULE OR (NOT (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")))
  message(STATUS "Visualization targets are not built on non-x86_64 or when built as module")
  set(ENABLE_VISUALIZATION OFF)
elseif (ENABLE_VISUALIZATION)
  message(STATUS "Visualization targets enabled")
else ()
  message(STATUS "Visualization targets disabled")
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
if (XMOTION_DEV_MODE)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
else ()
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
endif ()

if (XMOTION_DEV_MODE)
  message(STATUS "xmotion development mode enabled")
endif ()

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

## Enable test if requested
if (((NOT BUILD_AS_MODULE) AND BUILD_TESTING) OR XMOTION_DEV_MODE)
  enable_testing()
  include(GoogleTest)
  set(BUILD_TESTS ON)
  message(STATUS "xmotion tests will be built")
else ()
  set(BUILD_TESTS OFF)
  message(STATUS "xmotion tests will not be built")
endif ()

## Enable -fPIC for all targets
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

## Dependent libraries
add_subdirectory(third_party)

## Foundation (external). A parent superbuild may already provide it.
if (NOT TARGET xmotion::xmBase)
  find_package(xmBase 0.4.0 QUIET)
  if (NOT xmBase_FOUND)
    if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/third_party/xmBase/CMakeLists.txt)
      add_subdirectory(third_party/xmBase)
    else ()
      message(FATAL_ERROR "xmNavigation: xmBase not found — install it or init third_party/xmBase")
    endif ()
  endif ()
endif ()

## pnm/rapidcsv declare bare include paths (unexportable) and predate
## install support; rewrap and install their headers here.
set_target_properties(pnm PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/pnm/include>;$<INSTALL_INTERFACE:include>")
set_target_properties(rapidcsv PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/rapidcsv/src>;$<INSTALL_INTERFACE:include>")
install(DIRECTORY third_party/pnm/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES third_party/rapidcsv/src/rapidcsv.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if (XMOTION_WERROR)
  # Applied after third_party/xmBase so vendored code builds with its own
  # flags. -Wmaybe-uninitialized stays a visible warning but does not gate:
  # GCC's flow analysis has a documented false-positive record inside
  # libstdc++ headers (std::function/std::regex internals under -O).
  # -Warray-bounds / -Wstringop-overread likewise stay visible but do not
  # gate: GCC 12/13 fire them on Eigen's SSE vectorization of small
  # fixed-size matrices (16-byte __m128d loads from 8-byte 1x1 objects),
  # a documented upstream false positive.
  add_compile_options(-Wall -Wextra -Werror -Wno-error=maybe-uninitialized
      -Wno-error=array-bounds -Wno-error=stringop-overread)
  # Vendored header-only libraries are consumed from first-party TUs; mark
  # their include directories SYSTEM so their headers never gate our build.
  # (quickviz modules included: its headers reach first-party TUs the same
  # way, and quickviz builds with its own flags, not ours)
  foreach (_vendored stb rapidcsv pnm ctfsm core viewer canvas plot image imcore)
    if (TARGET ${_vendored})
      get_target_property(_inc ${_vendored} INTERFACE_INCLUDE_DIRECTORIES)
      set_target_properties(${_vendored} PROPERTIES
          INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_inc}")
      unset(_inc)
    endif ()
  endforeach ()
endif ()

## Vendored header-only libraries referenced by exported static targets
## must be resolvable from the export set (their headers install already).
install(TARGETS stb pnm rapidcsv stopwatch EXPORT xmNavigationTargets)

## Add source directories
add_subdirectory(src)

## Cross-module integration tests (composed algorithm pipelines against
## the simulation tier). Campaign-labeled tests are excluded from PR CI
## and run by the nightly workflow.
if (BUILD_TESTS)
  add_subdirectory(tests)
endif ()

## Aggregate interface target: consumers link a single `xmotion::xmNavigation` to pull
## in the XMotion algorithm libraries (estimation, control, planning).
add_library(xmNavigation INTERFACE)
target_link_libraries(xmNavigation INTERFACE
    navtypes
    estimation
    kinematics
    pid
    fsm
    model
    geometry
    decomp
    sampling
    map_processing
    lattice
    markov
    reachability)
add_library(xmotion::xmNavigation ALIAS xmNavigation)
install(TARGETS xmNavigation
    EXPORT xmNavigationTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

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
    xmNavigationConfigVersion.cmake
    VERSION "${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${${PROJECT_NAME}_VERSION_PATCH}"
    COMPATIBILITY SameMajorVersion)

install(EXPORT xmNavigationTargets
    FILE xmNavigationTargets.cmake
    NAMESPACE xmotion::
    DESTINATION lib/cmake/xmNavigation)

configure_file(cmake/xmNavigationConfig.cmake.in xmNavigationConfig.cmake @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/xmNavigationConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/xmNavigationConfigVersion.cmake"
    DESTINATION lib/cmake/xmNavigation)

# Packaging support
set(CPACK_DEBIAN_PACKAGE_NAME "libxmotion-navigation")
set(CPACK_PACKAGE_VENDOR "Ruixiang Du")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "XMotion Navigation - robot motion algorithms (planning, estimation, control)")
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/xmotion")
set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
# Algorithm-centric dependency set (ADR 0005): no hardware I/O libraries.
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libgl1-mesa-dev, libglfw3-dev, libcairo2-dev,
    libtbb-dev, libboost-all-dev, libeigen3-dev,
    libopencv-dev, libyaml-cpp-dev, libncurses-dev, libpcl-dev, libglm-dev")
set(CPACK_GENERATOR "DEB")
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Ruixiang Du (ruixiang.du@gmail.com)")
set(CPACK_SOURCE_IGNORE_FILES .git dist .*build.* /\\\\.DS_Store)
include(CPack)
