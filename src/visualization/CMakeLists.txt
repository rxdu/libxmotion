cmake_minimum_required(VERSION 2.8.11)
project(librav_vis2)

# cmake module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")

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

## Put all binary files into /bin and libraries into /lib
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

## Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
# find_package(DISLIN REQUIRED)
# include_directories(${DISLIN_INCLUDE_DIRS})

# # Add libraries
# set(VIS_LIB_SRC
#     #graph_vis.cpp # header only
#     qtree_vis.cpp
#     sgrid_vis.cpp
#     #rrt_vis.cpp
#     vis_utils.cpp
# )
# add_library(vis STATIC ${VIS_LIB_SRC})
# target_link_libraries(vis map geometry ${OpenCV_LIBS})

# Add executables
# add_executable(test_dislin tests/test_dislin.cpp)
# target_link_libraries(test_dislin ${DISLIN_LIBRARIES})

# find_package(PLplot REQUIRED)
# include_directories(${PLplot_INCLUDE_DIRS})

# add_executable(test_plplot tests/test_plplot.cpp tests/plstream.cc)
# target_link_libraries(test_plplot ${PLplot_LIBRARIES})

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

add_executable(test_vtk MACOSX_BUNDLE tests/test_vtk.cpp )

target_link_libraries(test_vtk ${VTK_LIBRARIES})
