# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(OpenCV REQUIRED)

# Add libraries
set(PATH_REPAIR_LIB_SRC
    quad_path_repair.cpp
)
add_library(path_repair STATIC ${PATH_REPAIR_LIB_SRC})
target_link_libraries(path_repair map planner ${OpenCV_LIBS} ${Octomap_LIBS} ${LCM_LIBRARIES})

# Add executables
add_executable(quadsim_pr_planner quadsim_planner.cpp)
target_link_libraries(quadsim_pr_planner map path_repair ${OpenCV_LIBS} ${LCM_LIBRARIES})

add_executable(graph_combiner graph_combiner.cpp)
target_link_libraries(graph_combiner map local3d ${Octomap_LIBS} ${LCM_LIBRARIES})
