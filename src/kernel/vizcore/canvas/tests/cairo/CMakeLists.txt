# Dependency libraries
# find_package(FLTK REQUIRED)

## tests
add_executable(test_cairo_x11 test_cairo_x11.cpp )
target_link_libraries(test_cairo_x11 canvas)

add_executable(test_cairo_draw test_cairo_draw.cpp)
target_link_libraries(test_cairo_draw canvas)

add_executable(test_cairo_grid test_cairo_grid.cpp)
target_link_libraries(test_cairo_grid canvas)

add_subdirectory(box)
add_subdirectory(flwin)