## Add libraries
set(GL3W_LIB_SRC
    GL/gl3w.c
)
add_library(imgui_gl3w STATIC ${GL3W_LIB_SRC})
# target_link_libraries(imgui glfw ${OPENGL_gl_LIBRARY})
target_include_directories(imgui_gl3w PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<INSTALL_INTERFACE:include>)
