# Dependency libraries
# find_package(FLTK REQUIRED)

## tests
add_executable(test_cairo_x11 test_cairo_x11.cpp )
target_link_libraries(test_cairo_x11 cairodraw)

add_executable(test_cairo_draw test_cairo_draw.cpp)
target_link_libraries(test_cairo_draw cairodraw)

add_executable(test_cairo_grid test_cairo_grid.cpp)
target_link_libraries(test_cairo_grid cairodraw)

add_subdirectory(box)
add_subdirectory(flwin)