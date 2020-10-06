## Dependency libraries
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)

## Add libraries
set(GLAD_SRC
    src/glad.c
)
add_library(glad STATIC ${GLAD_SRC})
target_link_libraries(glad -ldl ${OPENGL_gl_LIBRARY})
target_include_directories(glad PUBLIC
    $<BUILD_INTERFACE:${OPENGL_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)