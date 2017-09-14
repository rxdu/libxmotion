# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(OpenCV REQUIRED)

# Add libraries
set(PATH_REPAIR_LIB_SRC
    quad_path_repair.cpp
    sim_path_repair.cpp
    geo_mark_graph.cpp
    shortcut_eval.cpp
    sg_graph_planner.cpp
)
add_library(path_repair STATIC ${PATH_REPAIR_LIB_SRC})
target_link_libraries(path_repair map mission polyopt common vis ${OpenCV_LIBS} ${LCM_LIBRARIES})

set(PATH_REPAIR_SIM_SRC
    sim/virtual_quadrotor.cpp
)
add_library(pr_sim STATIC ${PATH_REPAIR_SIM_SRC})
target_link_libraries(pr_sim path_repair ${LCM_LIBRARIES})

# Add executables
#add_executable(graph_combine_exp tests/graph_combine_exp.cpp)
#target_link_libraries(graph_combine_exp map utils ${Octomap_LIBS} ${LCM_LIBRARIES})

#add_executable(test_combiner tests/test_combiner.cpp)
#target_link_libraries(test_combiner map geometry polyopt g3logger path_repair ${Octomap_LIBS} ${LCM_LIBRARIES})

#add_executable(test_combinerv2 tests/test_combiner_v2.cpp)
#target_link_libraries(test_combinerv2 map geometry polyopt g3logger path_repair ${Octomap_LIBS} ${LCM_LIBRARIES})

#add_executable(test_tracking tests/test_tracking.cpp)
#target_link_libraries(test_tracking polyopt ${LCM_LIBRARIES})

#add_executable(test_cubearray tests/test_cubearray.cpp)
#target_link_libraries(test_cubearray geometry)

#add_executable(test_cubearray_builder tests/test_cubearray_builder.cpp)
#target_link_libraries(test_cubearray_builder geometry  ${LCM_LIBRARIES})

#add_executable(graph_combine_exp tests/graph_combine_exp.cpp)
#target_link_libraries(graph_combine_exp map utils ${Octomap_LIBS} ${LCM_LIBRARIES})

#add_executable(test_calc tests/test_calc.cpp)
#target_link_libraries(test_calc path_repair map vis ${OpenCV_LIBS})

#add_executable(test_navfield tests/test_navfield.cpp)
#target_link_libraries(test_navfield map vis ${OpenCV_LIBS})

#add_executable(test_navfield2 tests/test_navfield2.cpp)
#target_link_libraries(test_navfield2 map vis path_repair ${OpenCV_LIBS})

#add_executable(test_navfield3 tests/test_navfield3.cpp)
#target_link_libraries(test_navfield3 map vis path_repair ${OpenCV_LIBS})

#add_executable(test_navfield4 tests/test_navfield4.cpp)
#target_link_libraries(test_navfield4 map vis path_repair ${OpenCV_LIBS} ${LCM_LIBRARIES})
