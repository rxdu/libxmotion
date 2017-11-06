cmake_minimum_required(VERSION 2.8.11)
project(librav_libs)

## Put all binary files into /bin and libraries into /lib
#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

## Add sub source directories
add_subdirectory(octomap-1.8.0)
add_subdirectory(googletest)

## UAVCAN library
# Include the Libuavcan CMake subproject
add_subdirectory(uavcan)

# Add Libuavcan include directories
target_include_directories(uavcan PUBLIC
	uavcan/libuavcan/include
	uavcan/libuavcan_drivers/linux/include
	uavcan/libuavcan/include/dsdlc_generated
)
