# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_pose_viewer test_pose_viewer.cpp)
target_link_libraries(test_pose_viewer viewers)