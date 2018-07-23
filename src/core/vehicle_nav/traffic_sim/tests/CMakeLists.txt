# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_trafficsim test_trafficsim.cpp)
target_link_libraries(test_trafficsim field trafficsim utilities)