## Dependency libraries
find_package(FLTK QUIET)
find_package(X11 QUIET)
find_package(Cairo QUIET)

if(FLTK_FOUND AND X11_FOUND AND CAIRO_FOUND)

include_directories(${FLTK_INCLUDE_DIR})
include_directories(${X11_INCLUDE_DIR})
include_directories(${CAIRO_INCLUDE_DIRS})

## Add libraries
set(CAIRODRAW_LIB_SRC
	src/cairo_canvas.cpp
	src/cairo_window.cpp
	src/cairo_draw.cpp
)
add_library(cairodraw STATIC ${CAIRODRAW_LIB_SRC})
target_link_libraries(cairodraw ${X11_LIBRARIES} ${FLTK_LIBRARIES} ${CAIRO_LIBRARIES})
# target_link_libraries(cairodraw ${X11_LIBRARIES} ${FLTK_LIBRARIES} ${CAIRO_LIBRARIES} ${OpenCV_LIBS})
target_include_directories(cairodraw PUBLIC	
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
add_subdirectory(tests)

endif()