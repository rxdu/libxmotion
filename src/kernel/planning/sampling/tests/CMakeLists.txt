# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_rvspace test_rvspace.cpp)
target_link_libraries(test_rvspace sampling utils)

add_executable(test_rng test_rng.cpp)
target_link_libraries(test_rng sampling)

add_executable(test_rrt test_rrt.cpp)
target_link_libraries(test_rrt sampling utils)

add_executable(test_kdtree test_kdtree.cpp)
target_link_libraries(test_kdtree sampling utils)