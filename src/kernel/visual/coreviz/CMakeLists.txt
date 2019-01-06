## Dependency libraries
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)

## Add libraries
# LightViz library
set(COREVIZ_LIB_SRC
	src/matrix_draw.cpp
	src/geometry_draw.cpp
	# src/curvilinear_grid_draw.cpp
)
add_library(coreviz STATIC ${COREVIZ_LIB_SRC})
target_link_libraries(coreviz cvdraw decomp graph sampling geometry pthread ${TBB_LIBRARIES})
target_include_directories(coreviz PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${TBB_INCLUDE_DIRS}>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(tests)
endif()