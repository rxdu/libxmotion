## Dependency libraries
find_package(GSL REQUIRED)

## Add libraries
set(TRAJECTORY_LIB_SRC
	src/trajectory.cpp
)
add_library(trajectory STATIC ${TRAJECTORY_LIB_SRC})
target_link_libraries(trajectory ${CGAL_LIBS} ${GSL_LIBRARIES})
target_include_directories(trajectory PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${GSL_INCLUDE_DIRS}>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(tests)
endif()