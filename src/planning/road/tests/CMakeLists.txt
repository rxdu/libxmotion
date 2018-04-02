# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/src)

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(lanelet_demo lanelet_demo.cpp)
target_link_libraries(lanelet_demo lanelet)

add_executable(lanelet_test lanelet_test.cpp)
target_link_libraries(lanelet_test lanelet)