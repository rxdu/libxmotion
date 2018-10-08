# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

## tests
add_executable(test_cairo_fltk driver.cpp CairoBox.cpp CairoBox.h)
target_link_libraries(test_cairo_fltk fastdraw)