## Dependency libraries
find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)

## Add libraries
set(POLYNOMIAL_LIB_SRC
	src/rpoly/rpoly_ak1.cpp
	src/polynomial.cpp
)
add_library(polynomial STATIC ${POLYNOMIAL_LIB_SRC})
target_link_libraries(polynomial ${GLOG_LIBRARIES})
target_include_directories(polynomial PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(tests)
endif()