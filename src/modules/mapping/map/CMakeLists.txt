## Dependency libraries
# find_package(Eigen3 REQUIRED)

## Add libraries
add_library(map 
    src/map_io.cpp
)
target_link_libraries(map cvdraw)
target_compile_definitions(map PUBLIC ENABLE_VISUAL)
target_include_directories(map PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
# if(BUILD_TESTS)
# 	add_subdirectory(test)
# endif()