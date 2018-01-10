# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Add libraries
add_library(spdlog INTERFACE)
target_include_directories(spdlog INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
	$<INSTALL_INTERFACE:include>)