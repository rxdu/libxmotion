# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_point_model test_point_model.cpp)
target_link_libraries(test_point_model lattice trafficviz)

add_executable(test_primitive_gen test_primitive_gen.cpp)
target_link_libraries(test_primitive_gen lattice trafficviz)

add_executable(test_lookup_table test_lookup_table.cpp)
target_link_libraries(test_lookup_table lattice trafficviz)

add_executable(test_state_lattice test_state_lattice.cpp)
target_link_libraries(test_state_lattice lattice trafficviz utilities)

add_executable(test_lattice_viz test_lattice_viz.cpp)
target_link_libraries(test_lattice_viz lattice utilities)