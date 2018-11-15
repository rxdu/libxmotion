cmake_minimum_required(VERSION 3.0.0)
project(gurobi_cpp)

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

set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -m64 -O")

## Put all binary files into /bin and libraries into /lib
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

## Dependency libraries
## Find Gurobi
find_package(GUROBI QUIET)
if (GUROBI_FOUND)
  message("-- Gurobi found, a local copy of libgurobi_c++.a will be built and used for better compatibility.")
  include_directories(${GUROBI_INCLUDE_DIRS})
endif()

## Add libraries
if(GUROBI_FOUND)
set(GUROBI_CPP_LIB_SRC
	Env.cpp
	SOS.cpp
	Var.cpp
	Model.cpp
	Column.cpp
	Constr.cpp
	LinExpr.cpp
	QConstr.cpp
	Callback.cpp
	QuadExpr.cpp
	Exception.cpp
	GenConstr.cpp
	TempConstr.cpp
)
add_library(gurobi_cpp STATIC ${GUROBI_CPP_LIB_SRC})
target_link_libraries(gurobi_cpp ${GUROBI_LIBRARIES})
target_include_directories(gurobi_cpp PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)
endif()