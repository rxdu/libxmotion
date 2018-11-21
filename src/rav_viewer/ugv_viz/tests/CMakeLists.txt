# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_ugvviz test_ugvviz.cpp)
target_link_libraries(test_ugvviz ugvviz)