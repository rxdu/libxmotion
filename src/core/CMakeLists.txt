cmake_minimum_required(VERSION 3.0.0)
project(librav_core)

## Add sub source directories
add_subdirectory(common)
add_subdirectory(interface)
add_subdirectory(utilities)

add_subdirectory(planning)
add_subdirectory(vehicle_nav)

add_subdirectory(visualization/lightviz)
# add_subdirectory(visualization/cinder)
