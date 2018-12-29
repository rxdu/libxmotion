# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_tree test_tree.cpp)
target_link_libraries(test_tree rrt)