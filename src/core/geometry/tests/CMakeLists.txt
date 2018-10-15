# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_polygon test_polygon.cpp)
target_link_libraries(test_polygon geometry)

add_executable(test_cspline test_cspline.cpp)
target_link_libraries(test_cspline geometry)