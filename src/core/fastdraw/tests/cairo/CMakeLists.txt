# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/src)

# Dependency libraries
# find_package(FLTK REQUIRED)
# include_directories(${FLTK_INCLUDE_DIR})

## tests
add_executable(test_cairo_x11 test_cairo_x11.cpp )
target_link_libraries(test_cairo_x11 fastdraw)

add_executable(test_cairo_draw test_cairo_draw.cpp)
target_link_libraries(test_cairo_draw fastdraw)

add_executable(test_cairo_grid test_cairo_grid.cpp)
target_link_libraries(test_cairo_grid fastdraw)

add_subdirectory(box)
add_subdirectory(flwin)