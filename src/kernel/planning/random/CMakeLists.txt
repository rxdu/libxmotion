## Dependency libraries
find_package(GSL REQUIRED)
find_package(Eigen3 REQUIRED)

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
	$<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(tests)
endif()