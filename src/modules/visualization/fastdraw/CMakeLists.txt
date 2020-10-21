## Dependency libraries
# find_package(OpenCV REQUIRED)

## Add libraries
set(FASTDRAW_LIB_SRC
	src/fastdraw.cpp
)
add_library(fastdraw STATIC ${FASTDRAW_LIB_SRC})
target_link_libraries(fastdraw raylib)
target_include_directories(fastdraw PUBLIC	
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
add_subdirectory(test)
