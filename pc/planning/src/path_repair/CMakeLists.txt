# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(OpenCV REQUIRED)

# Add libraries
set(PATH_REPAIR_LIB_SRC
    quad_path_repair.cpp
    geo_mark_graph.cpp
)
add_library(path_repair STATIC ${PATH_REPAIR_LIB_SRC})
target_link_libraries(path_repair map nav_field map3d polyopt quadflat common mission ${OpenCV_LIBS} ${Octomap_LIBS} ${LCM_LIBRARIES})

# Add executables
#add_executable(graph_combine_exp tests/graph_combine_exp.cpp)
#target_link_libraries(graph_combine_exp map local3d utils ${Octomap_LIBS} ${LCM_LIBRARIES})

#add_executable(test_combiner tests/test_combiner.cpp)
#target_link_libraries(test_combiner map geometry local3d polyopt quadflat g3logger path_repair ${Octomap_LIBS} ${LCM_LIBRARIES})

#add_executable(test_combinerv2 tests/test_combiner_v2.cpp)
#target_link_libraries(test_combinerv2 map geometry local3d polyopt quadflat g3logger path_repair ${Octomap_LIBS} ${LCM_LIBRARIES})

#add_executable(test_tracking tests/test_tracking.cpp)
#target_link_libraries(test_tracking polyopt quadflat ${LCM_LIBRARIES})

#add_executable(test_cubearray tests/test_cubearray.cpp)
#target_link_libraries(test_cubearray geometry)

#add_executable(test_cubearray_builder tests/test_cubearray_builder.cpp)
#target_link_libraries(test_cubearray_builder geometry  ${LCM_LIBRARIES})
