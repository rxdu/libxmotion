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
    waypoint_manager.cpp
)
add_library(path_repair STATIC ${PATH_REPAIR_LIB_SRC})
target_link_libraries(path_repair map local3d planner utils polyopt quadflat ${OpenCV_LIBS} ${Octomap_LIBS} ${LCM_LIBRARIES})

# Add executables
#add_executable(graph_combine_exp tests/graph_combine_exp.cpp)
#target_link_libraries(graph_combine_exp map local3d utils ${Octomap_LIBS} ${LCM_LIBRARIES})

add_executable(test_combiner tests/test_combiner.cpp)
target_link_libraries(test_combiner map geometry local3d polyopt quadflat utils ${Octomap_LIBS} ${LCM_LIBRARIES})

add_executable(wp_manager tests/wp_manager.cpp)
target_link_libraries(wp_manager path_repair ${LCM_LIBRARIES})

add_executable(test_waypoints tests/test_wpmanager.cpp)
target_link_libraries(test_waypoints map geometry local3d polyopt quadflat utils ${Octomap_LIBS} ${LCM_LIBRARIES})

add_executable(test_tracking tests/test_tracking.cpp)
target_link_libraries(test_tracking polyopt quadflat utils ${LCM_LIBRARIES})

add_executable(test_cubearray tests/test_cubearray.cpp)
target_link_libraries(test_cubearray geometry)
