cmake_minimum_required(VERSION 2.8.0)
project(quad_ctrl)

# Set compiler to use c++ 11 features
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

# Put all binary files into /bin and libraries into /lib
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

# Choose to enable g3log or not
set(ENABLE_G3LOG off)

# Check if g3log needs to be built
if(ENABLE_G3LOG)
	add_subdirectory(src/g3log_srcl)
	message( "-- Enable logging with g3log -- Yes" )
	message( "-- Deafult log directory: srcl_ctrl/control/hummingbird/quad_ctrl/data/log " )
else(ENABLE_G3LOG)
	message( "-- Enable logging with g3log -- No" )
endif(ENABLE_G3LOG)

# Add sub source directories
add_subdirectory(src/control)
add_subdirectory(src/vrep_sim)
add_subdirectory(src/apps)