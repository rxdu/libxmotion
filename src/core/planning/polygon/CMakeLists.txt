## Dependency libraries
find_package(CGAL REQUIRED)

## Add libraries
set(POLYGON_LIB_SRC
	src/polyline.cpp
	src/polygon.cpp
)
add_library(polygon STATIC ${POLYGON_LIB_SRC})
target_link_libraries(polygon ${CGAL_LIBS})
target_include_directories(polygon PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${CGAL_INCLUDE_DIRS}>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(tests)
endif()