# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_gui test_gui.cpp )
target_link_libraries(test_gui lightviz)

# add_executable(test_glfw_gui test_glfw_gui.cpp )
# target_link_libraries(test_glfw_gui lightviz glfw)

# add_executable(test_imglfw opengl3/main.cpp opengl3/gl3w.c opengl3/imgui_impl_glfw_gl3.cpp opengl3/imgui_impl_glfw_gl3.h)
# target_link_libraries(test_imglfw lightviz glfw)