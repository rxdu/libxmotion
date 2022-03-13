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
set(IMPLOT_SRC
    implot/implot.cpp
    implot/implot_items.cpp
    # implot/implot_demo.cpp
)
add_library(imcore ${IMGUI_CORE_SRC} ${IMGUI_BACKEND_SRC} ${IMPLOT_SRC})
target_link_libraries(imcore PUBLIC glfw OpenGL::GL ${GLFW3_LIBRARY} ${CMAKE_DL_LIBS})
target_compile_definitions(imcore PUBLIC "-DIMGUI_IMPL_OPENGL_LOADER_GL3W")
target_include_directories(imcore PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/imgui>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/imgui/examples/libs/gl3w>
    $<INSTALL_INTERFACE:include/imgui>
    $<INSTALL_INTERFACE:include/imgui/backends>
    $<INSTALL_INTERFACE:include/imgui/examples/libs/gl3w>)

add_subdirectory(sample)
