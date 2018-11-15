## Dependency libraries
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(TBB REQUIRED)
include_directories(${TBB_INCLUDE_DIRS})

## Add libraries
# LightViz library
set(LIGHTVIZ_LIB_SRC
	src/cartesian_canvas.cpp
	src/matrix_draw.cpp
	src/geometry_draw.cpp
	# src/curvilinear_grid_draw.cpp
	src/matrix_viz.cpp
	src/polygon_viz.cpp
	src/curve_viz.cpp
)
add_library(lightviz STATIC ${LIGHTVIZ_LIB_SRC})
target_link_libraries(lightviz decomp graph geometry canvas pthread ${TBB_LIBRARIES})
target_include_directories(lightviz PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(tests)
endif()