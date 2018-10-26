# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_grid_base test_grid_base.cpp)
target_link_libraries(test_grid_base decomp)

add_executable(test_newgrid test_newgrid.cpp)
target_link_libraries(test_newgrid decomp)

add_executable(test_dense_grid test_dense_grid.cpp)
target_link_libraries(test_dense_grid decomp lightviz)

add_executable(test_curvilinear_grid test_curvilinear_grid.cpp)
target_link_libraries(test_curvilinear_grid decomp lightviz)

add_executable(test_square_grid test_square_grid.cpp)
target_link_libraries(test_square_grid decomp lightviz)

# add_executable(test_decomposer test_decomposer.cpp)
# target_link_libraries(test_decomposer decomp lightviz)