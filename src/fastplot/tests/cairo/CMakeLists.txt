# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/src)

# Dependency libraries
# find_package(FLTK REQUIRED)
# include_directories(${FLTK_INCLUDE_DIR})

## tests
add_executable(test_cairo test_cairo.cpp )
target_link_libraries(test_cairo ${CAIRO_LIBRARIES} ${X11_LIBRARIES})

add_executable(test_cairo_win test_cairo_win.cpp)
target_link_libraries(test_cairo_win ${CAIRO_LIBRARIES} ${FLTK_LIBRARIES})

add_executable(test_cairo_fltk driver.cpp CairoBox.cpp CairoBox.h)
target_link_libraries(test_cairo_fltk ${CAIRO_LIBRARIES} ${FLTK_LIBRARIES})

add_executable(test_cairo_utils test_cairo_utils.cpp)
target_link_libraries(test_cairo_utils fastplot ${CAIRO_LIBRARIES} ${FLTK_LIBRARIES})