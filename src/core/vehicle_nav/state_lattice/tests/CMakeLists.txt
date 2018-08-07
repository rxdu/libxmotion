# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_lattice_gen test_lattice_gen.cpp)
target_link_libraries(test_lattice_gen lattice)

add_executable(test_lattice_manager test_lattice_manager.cpp)
target_link_libraries(test_lattice_manager lattice)