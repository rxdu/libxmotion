# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/src)

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_imageio test_imageio.cpp)
target_link_libraries(test_imageio utilities lightviz)
