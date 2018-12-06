# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# tests
add_executable(test_motion_manager test_motion_manager.cpp)
target_link_libraries(test_motion_manager cav_motion ugvnav_viz utils)

# add_executable(test_lookahead_zone test_lookahead_zone.cpp)
# target_link_libraries(test_lookahead_zone local_planner ugvnav_viz utils)