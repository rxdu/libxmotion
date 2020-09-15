## Find Eigen3
# find_package(Eigen3 REQUIRED)

# Add libraries
add_library(sensor_if INTERFACE)
target_link_libraries(sensor_if datalink)
target_include_directories(sensor_if INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include>)