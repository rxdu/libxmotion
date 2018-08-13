# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_cfield_viz test_cfield_viz.cpp)
target_link_libraries(test_cfield_viz tfield vehviz lightviz utilities)
