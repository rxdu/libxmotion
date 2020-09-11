cmake_minimum_required(VERSION 3.0.0)
project(imgui)

# VERSION V1.66b

## Dependency libraries
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)
# find_package(GLEW REQUIRED)
# find_package(GLUT REQUIRED)

## Add libraries
set(IMGUI_LIB_SRC
    imgui/imgui.cpp
    imgui/imgui_draw.cpp
    imgui/imgui_widgets.cpp
    imgui/imgui_demo.cpp
    # OpenGL + loader
    imgui/support/imgui_impl_glfw.cpp 
    # imgui/support/imgui_impl_opengl3.cpp
    # imgui/support/gl3w/GL/gl3w.c
)
add_library(imgui STATIC ${IMGUI_LIB_SRC})
target_link_libraries(imgui glfw ${OPENGL_gl_LIBRARY} ${GLEW_LIBRARIES} ${GLUT_glut_LIBRARYs} -ldl -lGLU)
target_include_directories(imgui PUBLIC
    $<BUILD_INTERFACE:${OPENGL_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/imgui>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/imgui/support>
    # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/imgui/support/gl3w>
    $<BUILD_INTERFACE:${GLEW_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${GLUT_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include>)