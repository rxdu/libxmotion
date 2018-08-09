# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_c2 test_c2.cpp)
target_link_libraries(test_c2 collision)

add_executable(test_checker test_checker.cpp)
target_link_libraries(test_checker collision)