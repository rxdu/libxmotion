# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

## tests
add_executable(SurfacePlot test_vtk_surface.cpp)
target_link_libraries(SurfacePlot ${VTK_LIBRARIES})