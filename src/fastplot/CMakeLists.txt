cmake_minimum_required(VERSION 3.7.2)
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
find_package(OpenCV REQUIRED)

## (Optional) Libraries for advanced plotting 
# Eigen3
find_package(Eigen3 QUIET)
if(EIGEN3_FOUND)
	include_directories(${EIGEN3_INCLUDE_DIR})
endif()
# VTK
find_package(VTK 7 QUIET)
if(VTK_FOUND)
	include(${VTK_USE_FILE})
endif()

## Add libraries
set(FASTPLOT_LIB_SRC
	src/image_plot.cpp
	src/map_plot.cpp
	src/graph_plot.cpp
	src/plot_utils.cpp
)

if(VTK_FOUND AND EIGEN3_FOUND)
	message(STATUS "Advanced plotting features will be built.")
	set(ADVANCED_PLOT_SRC
		src/surface_plot.cpp
	)
	set(ADV_DEPENDENT_LIBS
		${OpenCV_LIBS}
		${VTK_LIBRARIES}
	)
else()
	message(STATUS "Eigen3 or VTK 7+ is not found, fastplot will be built without advanced plotting.")
endif()
add_library(fastplot STATIC ${FASTPLOT_LIB_SRC} ${ADVANCED_PLOT_SRC})
target_link_libraries(fastplot geometry graph ${ADV_DEPENDENT_LIBS} ${OpenCV_LIBS})
target_include_directories(fastplot PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

# Add executables
add_subdirectory(tests)
