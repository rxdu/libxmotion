# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_sim_manager test_sim_manager.cpp)
target_link_libraries(test_sim_manager trafficsim utils)

add_executable(test_ego_state test_ego_state.cpp)
target_link_libraries(test_ego_state trafficsim utils)

# add_executable(test_trafficsim test_trafficsim.cpp)
# target_link_libraries(test_trafficsim tfield trafficsim utils)