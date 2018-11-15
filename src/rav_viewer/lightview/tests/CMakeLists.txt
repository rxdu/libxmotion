# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_lightview test_lightview.cpp)
target_link_libraries(test_lightview markov)

add_executable(test_imgui test_imgui.cpp)
target_link_libraries(test_imgui imgui)