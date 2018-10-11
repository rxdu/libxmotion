# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add executables
add_executable(test_jointcorridor test_jointcorridor.cpp)
target_link_libraries(test_jointcorridor quad_traj)

add_executable(test_trajectory_manager test_traj_manager.cpp)
target_link_libraries(test_trajectory_manager quad_traj)
