# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
# add_subdirectory(fltk)
# add_subdirectory(cairo)
# add_subdirectory(opencv)

# add_executable(test_cv_cairo test_cv_cairo.cpp)
# target_link_libraries(test_cv_cairo canvas)

add_executable(test_cv_canvas test_cv_canvas.cpp)
target_link_libraries(test_cv_canvas cvdraw)

add_executable(test_cv_drawmode test_cv_drawmode.cpp)
target_link_libraries(test_cv_drawmode cvdraw)