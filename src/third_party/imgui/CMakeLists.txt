cmake_minimum_required(VERSION 3.0.0)
project(imgui)

## Dependency libraries
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

## Add libraries
set(IMGUI_LIB_SRC
    imgui/imgui.cpp
    imgui/imgui_draw.cpp
    imgui/imgui_widgets.cpp
    imgui/imgui_demo.cpp
    # OpenGL + loader
    imgui/support/imgui_impl_glfw.cpp 
    imgui/support/imgui_impl_opengl3.cpp
)
add_library(imgui STATIC ${IMGUI_LIB_SRC})
target_link_libraries(imgui imgui_gl3w glfw ${OPENGL_gl_LIBRARY} -ldl)
target_include_directories(imgui PUBLIC
    $<BUILD_INTERFACE:${OPENGL_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/imgui>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/imgui/support>
	$<INSTALL_INTERFACE:include>)

add_subdirectory(imgui/support/gl3w)