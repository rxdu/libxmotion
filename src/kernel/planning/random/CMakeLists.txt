## Dependency libraries
find_package(GSL REQUIRED)

## Add libraries
set(RANDOM_LIB_SRC
	src/gaussian_sampler.cpp
	src/bigaussian_sampler.cpp
)
add_library(random STATIC ${RANDOM_LIB_SRC})
target_link_libraries(random ${CGAL_LIBS} ${GSL_LIBRARIES})
target_include_directories(random PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${GSL_INCLUDE_DIRS}>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(tests)
endif()