# Dependent libraries

## tests
# add_executable(test_lightview test_lightview.cpp)
# target_link_libraries(test_lightview imgui)

# add_executable(test_viewer test_viewer.cpp)
# target_link_libraries(test_viewer viewer)

add_executable(test_lightwin test_lightwin.cpp)
target_link_libraries(test_lightwin viewer)

add_executable(test_glfw test_glfw.cpp)
target_link_libraries(test_glfw viewer)

add_executable(test_imgui test_imgui.cpp)
target_link_libraries(test_imgui viewer)