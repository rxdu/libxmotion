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
  target_link_libraries(quadsim_pr_planner map path_repair ${OpenCV_LIBS} ${LCM_LIBRARIES})
endif()

add_executable(map_gen map_gen.cpp)
target_link_libraries(map_gen map vis ${OpenCV_LIBS})


