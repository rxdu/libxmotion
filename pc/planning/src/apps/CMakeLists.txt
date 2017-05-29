# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/src)

# Dependency libraries
find_package(OpenCV REQUIRED)
find_package(GUROBI)

# Add executables
if (NOT GUROBI_FOUND)
  message("Gurobi not found. If you want to use Gurobi, set the GUROBI_HOME variable")
else()
  add_executable(quadsim_pr_planner quadsim_planner.cpp)
    if (ENABLE_LOGGING)
      add_definitions(-DENABLE_G3LOG)
      target_link_libraries(quadsim_pr_planner map path_repair ctrl_utils g3logger ${OpenCV_LIBS} ${LCM_LIBRARIES})
    else(ENABLE_LOGGING)
      target_link_libraries(quadsim_pr_planner map path_repair ${OpenCV_LIBS} ${LCM_LIBRARIES})
    endif (ENABLE_LOGGING)
endif()

add_executable(planar_search planar_search.cpp)
target_link_libraries(planar_search geometry map vis ${OpenCV_LIBS})

add_executable(map_gen map_gen.cpp)
target_link_libraries(map_gen map vis ${OpenCV_LIBS})

add_executable(map_param map_param.cpp)
target_link_libraries(map_param geometry vis ${OpenCV_LIBS})

add_executable(local_analysis local_analysis.cpp)
target_link_libraries(local_analysis map vis nav_field ${OpenCV_LIBS})

add_executable(octomap_server_node octomap_server_node.cpp)
target_link_libraries(octomap_server_node local3d map geometry ${Octomap_LIBS} ${LCM_LIBRARIES})

add_executable(shortcut_analysis shortcut_analysis.cpp)
target_link_libraries(shortcut_analysis map vis nav_field ${OpenCV_LIBS})
