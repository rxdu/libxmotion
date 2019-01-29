# to use this Figure library 
# 1. possibility: include the directory which contains this CMakeLists.txt (e.g. by add_subdirectory(FigureDirectory)
# 2. possibility: include the directory containing the files figure.cpp and figure.hpp and modify your CMakeLists.txt as follows:
#                 add_library(Figure figure.cpp)
#                 add_executable(exec_name exec_source.cpp)
#                 target_link_libraries(exec_name Figure)

cmake_minimum_required(VERSION 2.8)
project(FigureClass)

add_library(Figure figure.cpp)
target_include_directories(Figure ${CMAKE_CURRENT_SOURCE_DIR})
