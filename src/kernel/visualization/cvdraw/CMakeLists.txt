## Dependency libraries
find_package(OpenCV REQUIRED)

## Add libraries
set(CVDRAW_LIB_SRC
	src/cv_colors.cpp
    src/color_maps.cpp
    src/cv_canvas.cpp
    src/cv_io.cpp
)
add_library(cvdraw STATIC ${CVDRAW_LIB_SRC})
target_link_libraries(cvdraw ${OpenCV_LIBS})
target_include_directories(cvdraw PUBLIC	
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
add_subdirectory(tests)
