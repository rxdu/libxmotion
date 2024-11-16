# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_pid test_pid.cpp)
target_link_libraries(test_pid pid)