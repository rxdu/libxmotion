## Add libraries
add_library(map_processor 
    src/pgm_map.cpp
)
target_link_libraries(map_processor PUBLIC interface pnm logging cvdraw)
target_compile_definitions(map_processor PUBLIC ENABLE_VISUAL)
target_include_directories(map_processor PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(test)
endif()