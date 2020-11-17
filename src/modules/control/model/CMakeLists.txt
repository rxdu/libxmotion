## Dependency libraries
# find_package(LIB_NAME REQUIRED)

## Add libraries
add_library(model INTERFACE)
target_link_libraries(model INTERFACE utils odeint)
target_include_directories(model INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>)

# Add executables
# if(BUILD_TESTING)
	add_subdirectory(tests)
# endif()