## Dependency libraries
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)

## Add libraries
# LightViz library
set(LIGHTVIZ_LIB_SRC
	src/matrix_viz.cpp
	src/polygon_viz.cpp
	src/curve_viz.cpp
)
add_library(lightviz STATIC ${LIGHTVIZ_LIB_SRC})
target_link_libraries(lightviz decomp graph sampling geometry coreviz pthread ${TBB_LIBRARIES})
target_include_directories(lightviz PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${TBB_INCLUDE_DIRS}>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(tests)
endif()