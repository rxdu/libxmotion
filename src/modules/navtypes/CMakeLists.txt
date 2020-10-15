# Dependency libraries
## Find Eigen3
find_package(Eigen3 REQUIRED)

# Add libraries
add_library(navtypes INTERFACE)
target_include_directories(navtypes INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include>)

