# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(app_sim_scenario1 app_sim_scenario1.cpp)
target_link_libraries(app_sim_scenario1 trafficsim)

add_executable(app_sim_scenario2 app_sim_scenario2.cpp)
target_link_libraries(app_sim_scenario2 trafficsim)

add_executable(app_sim_scenario3 app_sim_scenario3.cpp)
target_link_libraries(app_sim_scenario3 trafficsim)

add_executable(app_sim_scenario4 app_sim_scenario4.cpp)
target_link_libraries(app_sim_scenario4 trafficsim)