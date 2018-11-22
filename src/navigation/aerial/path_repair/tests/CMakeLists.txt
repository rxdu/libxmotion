# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add executables
# add_executable(test_map_request test_map_request.cpp)
# target_link_libraries(test_map_request  legacy vis datalink)

# add_executable(test_sim_sensor test_sim_sensor.cpp)
# target_link_libraries(test_sim_sensor pr_sim )

#add_executable(graph_combine_exp tests/graph_combine_exp.cpp)
#target_link_libraries(graph_combine_exp legacy utils ${Octomap_LIBS} datalink)

#add_executable(test_combiner tests/test_combiner.cpp)
#target_link_libraries(test_combiner legacy  polyopt g3logger path_repair ${Octomap_LIBS} datalink)

#add_executable(test_combinerv2 tests/test_combiner_v2.cpp)
#target_link_libraries(test_combinerv2 legacy  polyopt g3logger path_repair ${Octomap_LIBS} datalink)

#add_executable(test_tracking tests/test_tracking.cpp)
#target_link_libraries(test_tracking polyopt datalink)

#add_executable(test_cubearray tests/test_cubearray.cpp)
#target_link_libraries(test_cubearray )

#add_executable(test_cubearray_builder tests/test_cubearray_builder.cpp)
#target_link_libraries(test_cubearray_builder   datalink)

#add_executable(graph_combine_exp tests/graph_combine_exp.cpp)
#target_link_libraries(graph_combine_exp legacy utils ${Octomap_LIBS} datalink)

#add_executable(test_calc tests/test_calc.cpp)
#target_link_libraries(test_calc path_repair legacy vis)

#add_executable(test_navfield tests/test_navfield.cpp)
#target_link_libraries(test_navfield legacy vis)

#add_executable(test_navfield2 tests/test_navfield2.cpp)
#target_link_libraries(test_navfield2 legacy vis path_repair)

#add_executable(test_navfield3 tests/test_navfield3.cpp)
#target_link_libraries(test_navfield3 legacy vis path_repair)

add_executable(test_navfield4 test_navfield4.cpp)
target_link_libraries(test_navfield4 pr_sim legacy vis datalink)

add_executable(test_heading_policy test_heading_policy.cpp)
target_link_libraries(test_heading_policy legacy pr_sim vis datalink)