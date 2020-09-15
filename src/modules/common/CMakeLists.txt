# Dependency libraries
## Find Eigen3
find_package(Eigen3 REQUIRED)

# Add libraries
add_library(common INTERFACE)
target_include_directories(common INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include>)

