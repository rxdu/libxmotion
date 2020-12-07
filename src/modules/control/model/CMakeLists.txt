## Dependency libraries
find_package(Boost REQUIRED)

## Add libraries
add_library(model INTERFACE)
target_link_libraries(model INTERFACE utils ${Boost_LIBRARIES})
target_include_directories(model INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>)

# Add executables
# if(BUILD_TESTS)
	add_subdirectory(tests)
# endif()