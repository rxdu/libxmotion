cmake_minimum_required(VERSION 2.8.11)
project(librav_fastplot)

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

## Dependency libraries
# CImg is a single-head library, which is included in "libraries" folder
find_package(CImg REQUIRED)
include_directories(${CImg_INCLUDE_DIRS})

#find_package(VTK REQUIRED)
#include(${VTK_USE_FILE})

# # Add libraries
set(FASTPLOT_LIB_SRC
    src/image_plot.cpp
)
add_library(fastplot STATIC ${FASTPLOT_LIB_SRC})
target_link_libraries(fastplot ${CImg_SYSTEM_LIBS})
target_include_directories(fastplot PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

# Add executables
# add_executable(test_dislin tests/test_dislin.cpp)
# target_link_libraries(test_dislin ${DISLIN_LIBRARIES})

# find_package(PLplot REQUIRED)
# include_directories(${PLplot_INCLUDE_DIRS})

# add_executable(test_plplot tests/test_plplot.cpp tests/plstream.cc)
# target_link_libraries(test_plplot ${PLplot_LIBRARIES})

add_subdirectory(tests)
