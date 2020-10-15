# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# tests
add_executable(test_motion_manager test_motion_manager.cpp)
target_link_libraries(test_motion_manager mission trafficviz utils)

add_executable(test_route_planner test_route_planner.cpp)
target_link_libraries(test_route_planner mission trafficviz utils)