## Dependency libraries
find_package(CGAL REQUIRED)

## Add libraries
set(GEOMETRY_LIB_SRC
	src/polyline.cpp
	src/polygon.cpp
)
add_library(geometry STATIC ${GEOMETRY_LIB_SRC})
target_link_libraries(geometry ${CGAL_LIBS})
# target_compile_options(geometry PUBLIC "-frounding-math")
target_compile_definitions(geometry PUBLIC "-DCGAL_DISABLE_ROUNDING_MATH_CHECK")
target_include_directories(geometry PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${CGAL_INCLUDE_DIRS}>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(tests)
endif()