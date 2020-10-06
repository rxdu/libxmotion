## Dependency libraries
find_package(CGAL REQUIRED)
find_package(GSL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)

## Add libraries
set(GEOMETRY_LIB_SRC
	src/polyline.cpp
	src/polygon.cpp
	src/cspline.cpp
    src/parametric_curve.cpp
    # drawing
    src/matrix_draw.cpp
	src/geometry_draw.cpp
)
add_library(geometry STATIC ${GEOMETRY_LIB_SRC})
target_link_libraries(geometry cvdraw ${CGAL_LIBS} ${GSL_LIBRARIES} ${TBB_LIBRARIES})
# target_compile_options(geometry PUBLIC "-frounding-math")
target_compile_definitions(geometry PUBLIC "-DCGAL_DISABLE_ROUNDING_MATH_CHECK")
target_include_directories(geometry PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${CGAL_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${GSL_INCLUDE_DIRS}>
	$<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
# if(BUILD_TESTS)
	add_subdirectory(tests)
# endif()