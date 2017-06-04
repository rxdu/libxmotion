cmake_minimum_required(VERSION 2.8.11)
project(srcl_3rd_party_lib)

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

## Check if g3log needs to be built
if(ENABLE_LOGGING)
	add_subdirectory(g3log)
endif(ENABLE_LOGGING)

## Check if OMPL can be built
if(Boost_FOUND)
	#message(STATUS "OMPL will be built.")
	#add_subdirectory(ompl-1.2.0)
elseif(Boost_FOUND)
	message(STATUS "Boost 1.54 or higher is not found. OMPL will not be built.")
endif(Boost_FOUND)

## Add sub source directories
#add_subdirectory(eigen)
add_subdirectory(octomap-1.8.0)
add_subdirectory(googletest)
#add_subdirectory(ompl-1.2.1)
#add_subdirectory(libbot2)
