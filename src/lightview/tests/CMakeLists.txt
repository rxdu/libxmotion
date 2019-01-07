# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_lightview test_lightview.cpp)
target_link_libraries(test_lightview lightview)

add_executable(test_imgui test_imgui.cpp)
target_link_libraries(test_imgui imgui)

add_executable(test_traffic_viewer test_traffic_viewer.cpp)
target_link_libraries(test_traffic_viewer trafficsim_viewer)