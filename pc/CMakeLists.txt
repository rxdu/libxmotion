cmake_minimum_required(VERSION 2.8.11)
project(srcl_ctrl)

## Put all binary files into /bin and libraries into /lib
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set(ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

## Choose to enable g3log or not: on/off
set(ENABLE_G3LOG off)

if (ENABLE_G3LOG)
include_directories(${CMAKE_SOURCE_DIR}/third_party/g3log_srcl/src)
endif (ENABLE_G3LOG)

include_directories(${CMAKE_SOURCE_DIR}/third_party/octomap-1.8.0/octomap/include)
include_directories(${CMAKE_SOURCE_DIR}/third_party/octomap-1.8.0/octovis/include)
include_directories(${CMAKE_SOURCE_DIR}/third_party/octomap-1.8.0/dynamicEDT3D/include)
set(Octomap_LIBS octomap octomath)

## Check dependencies
find_package(VTK 7.0)

if(NOT VTK_FOUND)
	message(STATUS "GUI module will not be built because VTK cannot be found.")
endif(NOT VTK_FOUND)

find_package(PkgConfig REQUIRED)
pkg_search_module(LCM REQUIRED lcm)

if(LCM_FOUND)
	set(LCM_GENERATED_HEADER_PATH ${CMAKE_BINARY_DIR}/include)
	include_directories(${LCM_GENERATED_HEADER_PATH})
elseif(LCM_FOUND)
	message(STATUS "IPC is not supported without LCM. The control module and part of the planning module will not be built.")
endif(LCM_FOUND)

## Add sub source directories
if (LCM_FOUND)
	add_subdirectory(comm)
	add_subdirectory(control)
endif (LCM_FOUND)
add_subdirectory(planning)
add_subdirectory(third_party)

if (VTK_FOUND)
add_subdirectory(gui)
endif(VTK_FOUND)
