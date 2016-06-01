cmake_minimum_required(VERSION 2.8.11)
project(srcl_ctrl)

## Put all binary files into /bin and libraries into /lib
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set(ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

# Choose to enable g3log or not: on/off
set(ENABLE_G3LOG off)

if (ENABLE_G3LOG)
include_directories(${CMAKE_SOURCE_DIR}/third_party/g3log_srcl/src)
endif (ENABLE_G3LOG)

include_directories(${CMAKE_SOURCE_DIR}/third_party/octomap-1.8.0/octomap/include)
include_directories(${CMAKE_SOURCE_DIR}/third_party/octomap-1.8.0/octovis/include)
include_directories(${CMAKE_SOURCE_DIR}/third_party/octomap-1.8.0/dynamicEDT3D/include)
set(Octomap_LIBS octomap octomath)

set(LCM_GENERATED_HEADER_PATH ${CMAKE_BINARY_DIR}/include)

## Add sub source directories
add_subdirectory(comm)
add_subdirectory(control)
add_subdirectory(planning)
add_subdirectory(third_party)
