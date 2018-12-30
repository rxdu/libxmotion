# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_bigaussian test_bigaussian.cpp)
target_link_libraries(test_bigaussian random logging)