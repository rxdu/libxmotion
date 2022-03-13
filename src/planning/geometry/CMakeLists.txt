## Dependency libraries
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(TBB REQUIRED)

## Add libraries
add_library(geometry STATIC 
    src/polyline.cpp
	src/cubic_spline.cpp
    src/parametric_curve.cpp
    src/polygon.cpp
)
target_link_libraries(geometry cvdraw Eigen3::Eigen ${TBB_LIBRARIES})
target_compile_definitions(geometry PUBLIC ENABLE_VISUAL)
target_include_directories(geometry PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(test)
endif()