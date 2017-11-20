# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/src)

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_vtk test_vtk.cpp )
target_link_libraries(test_vtk ${VTK_LIBRARIES})

add_executable(test_vtk2 test_vtk2.cpp )
target_link_libraries(test_vtk2 ${VTK_LIBRARIES})

add_executable(test_surface test_surface.cpp )
target_link_libraries(test_surface ${VTK_LIBRARIES})