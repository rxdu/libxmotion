## Dependency libraries
find_package(OpenCV REQUIRED)

# find_package(FLTK REQUIRED)
# include_directories(${FLTK_INCLUDE_DIR})

# find_package(X11 REQUIRED)
# include_directories(${X11_INCLUDE_DIR})

# find_package(Cairo REQUIRED)
# include_directories(${CAIRO_INCLUDE_DIRS})

## Add libraries
set(CVDRAW_LIB_SRC
	src/cv_colors.cpp
    src/color_maps.cpp
    src/cv_canvas.cpp
    src/cv_draw.cpp
)
add_library(cvdraw STATIC ${CVDRAW_LIB_SRC})
# target_link_libraries(canvas ${X11_LIBRARIES} ${FLTK_LIBRARIES} ${CAIRO_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(cvdraw ${OpenCV_LIBS})
target_include_directories(cvdraw PUBLIC	
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
add_subdirectory(tests)
