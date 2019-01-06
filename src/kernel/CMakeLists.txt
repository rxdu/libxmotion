cmake_minimum_required(VERSION 3.0.0)
project(librav_core)

## Add sub source directories
add_subdirectory(common)
add_subdirectory(logging)
add_subdirectory(device)
add_subdirectory(control)
add_subdirectory(decision)
add_subdirectory(planning)
add_subdirectory(datalink)
add_subdirectory(utilities)
add_subdirectory(visualization)

add_subdirectory(examples)

# to be deprecated
add_subdirectory(legacy)
