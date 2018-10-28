# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_basic_tree test_basic_tree.cpp)
target_link_libraries(test_basic_tree tree)

add_executable(test_unique_tree test_unique_tree.cpp)
target_link_libraries(test_unique_tree tree)