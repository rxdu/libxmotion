cmake_minimum_required(VERSION 2.8.11)
project(librav_core)

## Add sub source directories
add_subdirectory(common)
add_subdirectory(interface)
add_subdirectory(utilities)
add_subdirectory(planning)
add_subdirectory(visualization/lightviz)
add_subdirectory(visualization/cinder)
