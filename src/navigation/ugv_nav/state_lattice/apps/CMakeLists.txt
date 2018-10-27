# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(gen_lookup_table gen_lookup_table.cpp)
target_link_libraries(gen_lookup_table lattice ugvnav_viz)
