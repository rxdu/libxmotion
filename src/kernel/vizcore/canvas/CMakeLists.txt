## Dependency libraries
find_package(OpenCV REQUIRED)

# find_package(FLTK REQUIRED)
# include_directories(${FLTK_INCLUDE_DIR})

# find_package(X11 REQUIRED)
# include_directories(${X11_INCLUDE_DIR})

# find_package(Cairo REQUIRED)
# include_directories(${CAIRO_INCLUDE_DIRS})

## Add libraries
set(FASTDRAW_LIB_SRC
	# Cairo-based drawing
	# src/cairo_canvas.cpp
	# src/cairo_window.cpp
	# src/cairo_draw.cpp
	# OpenCV-based drawing
	src/cv_draw.cpp
)

add_library(canvas STATIC ${FASTDRAW_LIB_SRC})
# target_link_libraries(canvas ${X11_LIBRARIES} ${FLTK_LIBRARIES} ${CAIRO_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(canvas ${OpenCV_LIBS})
target_include_directories(canvas PUBLIC	
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
add_subdirectory(tests)
