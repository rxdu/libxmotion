# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_image_viz test_image_viz.cpp)
target_link_libraries(test_image_viz lightviz)

add_executable(test_grid_viz test_grid_viz.cpp )
target_link_libraries(test_grid_viz lightviz)

add_executable(test_polygon_viz test_polygon_viz.cpp )
target_link_libraries(test_polygon_viz lightviz)

add_executable(test_cspline_viz test_cspline_viz.cpp )
target_link_libraries(test_cspline_viz lightviz)

add_executable(test_field_viz test_field_viz.cpp )
target_link_libraries(test_field_viz lightviz)

add_executable(test_pcurve_viz test_pcurve_viz.cpp)
target_link_libraries(test_pcurve_viz lightviz)

add_executable(test_geogrid_viz test_geogrid_viz.cpp)
target_link_libraries(test_geogrid_viz lightviz)

# add_executable(test_glfw_gui test_glfw_gui.cpp )
# target_link_libraries(test_glfw_gui lightviz glfw)

# add_executable(test_imglfw opengl3/main.cpp opengl3/gl3w.c opengl3/imgui_impl_glfw_gl3.cpp opengl3/imgui_impl_glfw_gl3.h)
# target_link_libraries(test_imglfw lightviz glfw)
