# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_trajectory test_trajectory.cpp)
target_link_libraries(test_trajectory trajectory)