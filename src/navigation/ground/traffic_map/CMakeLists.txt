## Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## Add libraries
set(TRAFFICMAP_LIB_SRC
	src/traffic_channel.cpp
	src/traffic_map.cpp
)
add_library(traffic STATIC ${TRAFFICMAP_LIB_SRC})
target_link_libraries(traffic roadmap utils)
target_include_directories(traffic PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
if(BUILD_TESTS)
	# add_subdirectory(tests)
endif()