## Dependency libraries
# find_package(LIB_NAME REQUIRED)

## Add libraries
add_library(pid INTERFACE)
target_link_libraries(pid INTERFACE ctrl_if)
target_include_directories(pid INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include>)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(tests)
endif()