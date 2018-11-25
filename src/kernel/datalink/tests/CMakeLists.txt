# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_lcmlink test_lcmlink.cpp)
target_link_libraries(test_lcmlink datalink)