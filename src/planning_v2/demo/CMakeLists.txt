# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Add executables
## demo
add_executable(basic_example basic_example.cpp)
target_link_libraries(basic_example graph)
