cmake_minimum_required(VERSION 2.8.11)
project(srcl_control)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

## Add sub source directories
add_subdirectory(src/ctrl_utils)
add_subdirectory(src/quad_ctrl)
add_subdirectory(src/vrep_sim)
add_subdirectory(src/apps)
