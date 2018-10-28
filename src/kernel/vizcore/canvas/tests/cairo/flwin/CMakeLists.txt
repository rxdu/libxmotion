# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

## tests
add_executable(test_cairo_flwin test_cairo_flwin.cpp)
target_link_libraries(test_cairo_flwin canvas)