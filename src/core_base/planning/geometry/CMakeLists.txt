## Dependency libraries
find_package(CGAL REQUIRED)
find_package(GSL REQUIRED)

## Add libraries
set(GEOMETRY_LIB_SRC
	src/polyline.cpp
	src/polygon.cpp
	src/cspline.cpp
	src/parametric_curve.cpp
)
add_library(geometry STATIC ${GEOMETRY_LIB_SRC})
target_link_libraries(geometry ${CGAL_LIBS} ${GSL_LIBRARIES})
# target_compile_options(geometry PUBLIC "-frounding-math")
target_compile_definitions(geometry PUBLIC "-DCGAL_DISABLE_ROUNDING_MATH_CHECK")
target_include_directories(geometry PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${CGAL_INCLUDE_DIRS}>
	$<BUILD_INTERFACE:${GSL_INCLUDE_DIRS}>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(tests)
endif()