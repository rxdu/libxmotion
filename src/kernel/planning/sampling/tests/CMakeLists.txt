# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_tree test_tree.cpp)
target_link_libraries(test_tree sampling)

add_executable(test_rvspace test_rvspace.cpp)
target_link_libraries(test_rvspace sampling)

add_executable(test_rng test_rng.cpp)
target_link_libraries(test_rng sampling)