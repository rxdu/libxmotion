## Dependency libraries
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

## Add libraries
set(IMGUI_CORE_SRC
    imgui/imgui.cpp
    imgui/imgui_draw.cpp
    imgui/imgui_widgets.cpp
    imgui/imgui_tables.cpp
    imgui/imgui_demo.cpp
)
set(IMGUI_BACKEND_SRC
    imgui/backends/imgui_impl_glfw.cpp 
    imgui/backends/imgui_impl_opengl3.cpp
)
set(GL_LOADER_SRC
    imgui/examples/libs/gl3w/GL/gl3w.c
)
add_library(imgui ${IMGUI_CORE_SRC} ${IMGUI_BACKEND_SRC})
target_link_libraries(imgui PUBLIC glfw OpenGL::GL ${GLFW3_LIBRARY} ${CMAKE_DL_LIBS})
target_compile_definitions(imgui PUBLIC "IMGUI_IMPL_OPENGL_LOADER_GL3W" PRIVATE "IMGUI_DEFINE_MATH_OPERATORS")
target_include_directories(imgui PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/imgui>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/imgui/examples/libs/gl3w>
    $<INSTALL_INTERFACE:include/imgui>
    $<INSTALL_INTERFACE:include/imgui/backends>
    $<INSTALL_INTERFACE:include/imgui/examples/libs/gl3w>)

set(IMPLOT_SRC
    implot/implot.cpp
    implot/implot_items.cpp
    # implot/implot_demo.cpp
)
add_library(implot ${IMPLOT_SRC})
target_link_libraries(implot PUBLIC imgui)
target_include_directories(implot PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include/imgui>)

add_library(imcore INTERFACE)
target_link_libraries(imcore INTERFACE imgui implot)

add_subdirectory(sample)
