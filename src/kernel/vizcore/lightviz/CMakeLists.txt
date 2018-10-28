## Dependency libraries
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

## Add libraries
# LightViz library
set(LIGHTVIZ_LIB_SRC
	src/cartesian_canvas.cpp
	src/matrix_draw.cpp
	src/geometry_draw.cpp
	src/matrix_viz.cpp
	src/polygon_viz.cpp
	src/curve_viz.cpp
)
add_library(lightviz STATIC ${LIGHTVIZ_LIB_SRC})
target_link_libraries(lightviz decomp graph geometry canvas)
target_include_directories(lightviz PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(tests)
endif()