## tests
add_executable(test_cairo_fltk driver.cpp CairoBox.cpp CairoBox.h)
target_link_libraries(test_cairo_fltk canvas)