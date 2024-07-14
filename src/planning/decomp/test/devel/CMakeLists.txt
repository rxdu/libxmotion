# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
if (ENABLE_VISUALIZATION)
  add_executable(test_curvilinear_grid test_curvilinear_grid.cpp)
  target_link_libraries(test_curvilinear_grid decomp)

  add_executable(test_square_grid test_square_grid.cpp)
  target_link_libraries(test_square_grid decomp)

  add_executable(test_dense_grid test_dense_grid.cpp)
  target_link_libraries(test_dense_grid decomp)
endif ()

# add_executable(test_grid_base test_grid_base.cpp)
# target_link_libraries(test_grid_base decomp)

# add_executable(test_decomp_memory test_decomp_memory.cpp)
# target_link_libraries(test_decomp_memory decomp)

# drawing
# add_executable(test_sgrid_draw test_sgrid_draw.cpp)
# target_link_libraries(test_sgrid_draw decomp)

# add_executable(test_cgrid_draw test_cgrid_draw.cpp)
# target_link_libraries(test_cgrid_draw decomp)

# add_executable(test_sgrid_viz test_sgrid_viz.cpp )
# target_link_libraries(test_sgrid_viz decomp)