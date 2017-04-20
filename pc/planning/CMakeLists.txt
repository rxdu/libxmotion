cmake_minimum_required(VERSION 2.8.11)
project(srcl_planning)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules")

## Set compiler to use c++ 11 features
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
	message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++14 or C++11 support. Please use a different C++ compiler.")
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

## Conditional add sources
find_package(GUROBI)
if (NOT GUROBI_FOUND)
  message("Gurobi not found. If you want to use Gurobi, set the GUROBI_HOME variable")
else()
  message("Gurobi found")
  include_directories(${GUROBI_INCLUDE_DIRS})
    
  add_subdirectory(src/polyopt)        # <-----------------------------------
  add_subdirectory(src/quad_flat)      # <-----------------------------------
  add_subdirectory(src/path_repair)    # <-----------------------------------
endif()

## Add other sub source directories
add_subdirectory(src/geometry)
#add_subdirectory(src/graph)
add_subdirectory(src/map)
add_subdirectory(src/local3d)
add_subdirectory(src/vis)
add_subdirectory(src/rrtstar)
#add_subdirectory(src/planner)
#add_subdirectory(src/motion_client)
add_subdirectory(src/examples)
add_subdirectory(src/mission)
add_subdirectory(src/apps)
add_subdirectory(src/nav_field)

if(BUILD_TESTS)
    add_subdirectory(src/gtests)
endif()
