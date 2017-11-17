# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/src)

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_create_field test_create_field.cpp)
target_link_libraries(test_create_field field)

add_executable(test_field test_field.cpp)
target_link_libraries(test_field field ${LCM_LIBRARIES}) 