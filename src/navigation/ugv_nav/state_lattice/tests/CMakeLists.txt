# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_point_model test_point_model.cpp)
target_link_libraries(test_point_model lattice ugvnav_viz)

add_executable(test_primitive_gen test_primitive_gen.cpp)
target_link_libraries(test_primitive_gen lattice ugvnav_viz)

add_executable(test_lookup_table test_lookup_table.cpp)
target_link_libraries(test_lookup_table lattice ugvnav_viz)