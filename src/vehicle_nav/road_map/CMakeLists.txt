## Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## Add libraries
set(ROADMAP_LIB_SRC
	src/road_coordinate.cpp
	src/road_map.cpp
)
add_library(roadmap STATIC ${ROADMAP_LIB_SRC})
target_link_libraries(roadmap decomp lanelet geometry)
target_include_directories(roadmap PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
if(BUILD_TESTS)
add_subdirectory(tests)
endif()