## Dependency libraries
find_package(glfw3 REQUIRED)

## Add libraries
set(IMGUI_CORE_SRC
    imgui.cpp
    imgui_draw.cpp
    imgui_widgets.cpp
    imgui_demo.cpp
)
set(IMGUI_BACKEND_SRC
    examples/imgui_impl_glfw.cpp 
    examples/imgui_impl_opengl3.cpp
)
add_library(imgui STATIC ${IMGUI_CORE_SRC} ${IMGUI_BACKEND_SRC})
target_link_libraries(imgui glfw glad ${GLFW3_LIBRARY})
target_compile_definitions(imgui PRIVATE "-DIMGUI_IMPL_OPENGL_LOADER_GLAD")
target_include_directories(imgui PUBLIC
    $<BUILD_INTERFACE:${GLFW3_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/examples>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)