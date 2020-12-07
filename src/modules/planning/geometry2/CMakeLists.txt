## Dependency libraries
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)

## Add libraries
add_library(geometry2 STATIC 
    src/polyline.cpp
	src/cubic_spline.cpp
    src/parametric_curve.cpp
)
target_link_libraries(geometry2 cvdraw ${TBB_LIBRARIES})
target_compile_definitions(geometry2 PUBLIC ENABLE_VISUAL)
target_include_directories(geometry2 PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(test)
endif()