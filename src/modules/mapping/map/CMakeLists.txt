## Dependency libraries
find_package(Eigen3 REQUIRED NO_MODULE)

## Add libraries
add_library(map 
    src/pgm_map.cpp
)
target_link_libraries(map Eigen3::Eigen cvdraw)
target_compile_definitions(map PUBLIC ENABLE_VISUAL)
target_include_directories(map PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(test)
endif()