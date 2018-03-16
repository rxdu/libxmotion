# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/src)

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_imgplot test_imgplot.cpp )
target_link_libraries(test_imgplot fastplot)

add_executable(test_mapplot test_mapplot.cpp )
target_link_libraries(test_mapplot fastplot)

add_executable(test_surfplot test_surfplot.cpp )
target_link_libraries(test_surfplot fastplot)

add_executable(test_contourplot test_contourplot.cpp )
target_link_libraries(test_contourplot fastplot)

add_executable(test_fieldplot test_fieldplot.cpp )
target_link_libraries(test_fieldplot fastplot)

add_executable(test_graphplot test_graphplot.cpp )
target_link_libraries(test_graphplot fastplot planner)

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

# find_package(DISLIN REQUIRED GLUT)
# include_directories(${DISLIN_INCLUDE_DIRS})

# add_executable(test_dislin test_dislin.cpp )
# target_link_libraries(test_dislin ${DISLIN_LIBRARIES})

# add_executable(test_vtk test_vtk.cpp )
# target_link_libraries(test_vtk ${VTK_LIBRARIES})

# add_executable(test_vtk2 test_vtk2.cpp )
# target_link_libraries(test_vtk2 ${VTK_LIBRARIES})

# add_executable(test_surfaceplot test_surfaceplot.cpp )
# target_link_libraries(test_surfaceplot ${VTK_LIBRARIES})

# find_package(Boost REQUIRED)
# include_directories(${Boost_INCLUDE_DIRS})
# add_executable(matplot_example matplot/examples.cpp matplot/matplot.cpp matplot/matplot.h matplot/mtl4_dims.h)
# target_link_libraries(matplot_example ${VTK_LIBRARIES} )

add_subdirectory(fltk)
add_subdirectory(cairo)