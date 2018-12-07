## Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## Add libraries
set(CAV_COMMON_LIB_SRC
    src/vehicle_state.cpp
)
add_library(cav_common STATIC ${CAV_COMMON_LIB_SRC})
target_link_libraries(cav_common common geometry utils)
target_include_directories(cav_common PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
if(BUILD_TESTS)
	# add_subdirectory(tests)
endif()