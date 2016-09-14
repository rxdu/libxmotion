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
)
add_library(path_repair STATIC ${PATH_REPAIR_LIB_SRC})
target_link_libraries(path_repair map local3d planner utils polyopt quadflat ${OpenCV_LIBS} ${Octomap_LIBS} ${LCM_LIBRARIES})

# Add executables
add_executable(quadsim_pr_planner quadsim_planner.cpp)
target_link_libraries(quadsim_pr_planner map path_repair ${OpenCV_LIBS} ${LCM_LIBRARIES})

add_executable(graph_combine_exp graph_combine_exp.cpp)
target_link_libraries(graph_combine_exp map local3d utils ${Octomap_LIBS} ${LCM_LIBRARIES})

add_executable(test_combiner test_combiner.cpp)
target_link_libraries(test_combiner map geometry local3d utils ${Octomap_LIBS} ${LCM_LIBRARIES})
