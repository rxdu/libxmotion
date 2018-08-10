# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
# add_executable(test_lattice_gen test_lattice_gen.cpp)
# target_link_libraries(test_lattice_gen lattice)

# add_executable(test_lattice_manager test_lattice_manager.cpp)
# target_link_libraries(test_lattice_manager lattice)

add_executable(test_veh_planner test_lattice_planner.cpp)
target_compile_options(test_veh_planner PUBLIC "-frounding-math")
target_link_libraries(test_veh_planner lattice_plan tfield lightviz utilities)

# add_executable(test_lattice_graph test_lattice_graph.cpp)
# target_link_libraries(test_lattice_graph lattice)