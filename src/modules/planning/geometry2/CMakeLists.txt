## Dependency libraries
# find_package(CGAL REQUIRED)
# find_package(GSL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)

## Add libraries
add_library(geometry2 STATIC 
    src/polyline.cpp
	# src/polygon.cpp
	# src/cspline.cpp
    # src/parametric_curve.cpp
)
target_link_libraries(geometry2 cvdraw ${TBB_LIBRARIES})
# target_compile_options(geometry PUBLIC "-frounding-math")
# target_compile_definitions(geometry2 PUBLIC "-DCGAL_DISABLE_ROUNDING_MATH_CHECK")
target_include_directories(geometry2 PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
# if(BUILD_TESTS)
	add_subdirectory(test)
# endif()