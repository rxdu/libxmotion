# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# tests
add_executable(app_mission_control app_mission_control.cpp)
target_link_libraries(app_mission_control cav_motion utils)

add_executable(app_motion_manager app_motion_manager.cpp)
target_link_libraries(app_motion_manager cav_motion utils)