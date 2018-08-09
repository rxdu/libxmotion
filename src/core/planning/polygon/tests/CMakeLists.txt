# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_polygon test_polygon.cpp)
target_link_libraries(test_polygon polygon)