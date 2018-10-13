cmake_minimum_required(VERSION 3.0.0)
project(librav_core)

## Add sub source directories
add_subdirectory(common)
add_subdirectory(interface)
add_subdirectory(utilities)

add_subdirectory(model)
add_subdirectory(geometry)
add_subdirectory(planning)
add_subdirectory(lightviz)

add_subdirectory(examples)
