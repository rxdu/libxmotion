# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# find_package(MathGL2 REQUIRED GLUT)
# include_directories(${MATHGL2_INCLUDE_DIRS})

add_executable(test_mathgl test_mathgl.cpp )
target_link_libraries(test_mathgl fastplot ${MATHGL2_LIBRARIES})

add_executable(test_mathgl_wrapper test_mathgl_wrapper.cpp )
target_link_libraries(test_mathgl_wrapper fastplot)

add_executable(test_mathgl_surf test_mathgl_surf.cpp )
target_link_libraries(test_mathgl_surf fastplot)

add_executable(test_mathgl_2d test_mathgl_2d.cpp )
target_link_libraries(test_mathgl_2d fastplot)

add_executable(test_mathgl_qt test_mathgl_qt.cpp )
target_link_libraries(test_mathgl_qt fastplot)

# add_subdirectory(fltk)