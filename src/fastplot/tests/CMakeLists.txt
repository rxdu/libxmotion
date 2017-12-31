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

add_executable(test_graphplot test_graphplot.cpp )
target_link_libraries(test_graphplot fastplot planner)

# add_executable(test_vtk test_vtk.cpp )
# target_link_libraries(test_vtk ${VTK_LIBRARIES})

# add_executable(test_vtk2 test_vtk2.cpp )
# target_link_libraries(test_vtk2 ${VTK_LIBRARIES})

# add_executable(test_surface test_surface.cpp )
# target_link_libraries(test_surface ${VTK_LIBRARIES})

# add_executable(test_mathgl test_mathgl.cpp )
# target_link_libraries(test_mathgl ${MATHGL2_LIBRARIES})