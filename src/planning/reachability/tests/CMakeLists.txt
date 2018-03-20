# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_dynamics test_dynamics.cpp)
target_link_libraries(test_dynamics reachability)