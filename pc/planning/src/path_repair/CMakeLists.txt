# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(OpenCV REQUIRED)
find_package(GUROBI REQUIRED)
if (NOT GUROBI_FOUND)
  message("Gurobi not found. If you want to use Gurobi, set the GUROBI_HOME variable")
else()
  message("Gurobi found")
  include_directories(${GUROBI_INCLUDE_DIRS})
endif()

# Add libraries
set(PATH_REPAIR_LIB_SRC
    quad_path_repair.cpp
    geo_mark_graph.cpp
    #waypoint_manager.cpp
)
add_library(path_repair STATIC ${PATH_REPAIR_LIB_SRC})
if (ENABLE_LOGGING)
  add_definitions(-DENABLE_G3LOG)
  target_link_libraries(path_repair map nav_field local3d utils polyopt quadflat mission ctrl_utils g3logger ${OpenCV_LIBS} ${Octomap_LIBS} ${LCM_LIBRARIES})
else(ENABLE_LOGGING)
  target_link_libraries(path_repair map nav_field local3d utils polyopt quadflat mission ${OpenCV_LIBS} ${Octomap_LIBS} ${LCM_LIBRARIES})
endif (ENABLE_LOGGING)

# Add executables
#add_executable(graph_combine_exp tests/graph_combine_exp.cpp)
#target_link_libraries(graph_combine_exp map local3d utils ${Octomap_LIBS} ${LCM_LIBRARIES})

#add_executable(test_combiner tests/test_combiner.cpp)
#add_definitions(-DENABLE_G3LOG)
#target_link_libraries(test_combiner map geometry local3d polyopt quadflat utils g3logger path_repair ${Octomap_LIBS} ${LCM_LIBRARIES})

#add_executable(test_combinerv2 tests/test_combiner_v2.cpp)
#add_definitions(-DENABLE_G3LOG)
#target_link_libraries(test_combinerv2 map geometry local3d polyopt quadflat utils g3logger path_repair ${Octomap_LIBS} ${LCM_LIBRARIES})

#add_executable(test_tracking tests/test_tracking.cpp)
#target_link_libraries(test_tracking polyopt quadflat utils ${LCM_LIBRARIES})

add_executable(test_cubearray tests/test_cubearray.cpp)
target_link_libraries(test_cubearray geometry)

#add_executable(test_cubearray_builder tests/test_cubearray_builder.cpp)
#target_link_libraries(test_cubearray_builder geometry  ${LCM_LIBRARIES})
