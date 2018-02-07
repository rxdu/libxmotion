# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Add libraries
add_library(librav_if INTERFACE)
target_include_directories(librav_if INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
	$<INSTALL_INTERFACE:include>)