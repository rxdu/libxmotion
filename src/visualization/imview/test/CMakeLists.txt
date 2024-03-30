## base
add_executable(test_window test_window.cpp)
target_link_libraries(test_window imview)

add_executable(test_viewer test_viewer.cpp)
target_link_libraries(test_viewer imview)

add_executable(test_panel test_panel.cpp)
target_link_libraries(test_panel imview)

## implot
add_executable(test_plot_buffer test_plot_buffer.cpp)
target_link_libraries(test_plot_buffer PRIVATE imview)

## opengl
add_executable(test_opengl test_opengl.cpp)
target_link_libraries(test_opengl imview)

## cairo
add_executable(test_cairo_context test_cairo_context.cpp)
target_link_libraries(test_cairo_context PRIVATE imview)

add_executable(test_cairo_widget test_cairo_widget.cpp)
target_link_libraries(test_cairo_widget imview)

add_executable(test_cairo_draw test_cairo_draw.cpp)
target_link_libraries(test_cairo_draw imview)

add_executable(test_cairo_normalize test_cairo_normalize.cpp)
target_link_libraries(test_cairo_normalize imview)

