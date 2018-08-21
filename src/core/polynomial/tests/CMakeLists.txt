# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_polynomial test_polynomial.cpp)
target_link_libraries(test_polynomial polynomial)