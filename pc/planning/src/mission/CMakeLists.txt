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
set(MISSION_LIB_SRC
    mission_utils.cpp
    mission_tracker.cpp
    trajectory_generator.cpp
)
add_library(mission STATIC ${MISSION_LIB_SRC})
target_link_libraries(mission map local3d planner utils polyopt quadflat ${OpenCV_LIBS} ${Octomap_LIBS} ${LCM_LIBRARIES})

# Add executables
add_executable(wp_manager tests/wp_manager.cpp)
target_link_libraries(wp_manager mission ${LCM_LIBRARIES})

#add_executable(test_waypoints tests/test_wpmanager.cpp)
#target_link_libraries(test_waypoints mission map geometry local3d polyopt quadflat utils ${Octomap_LIBS} ${LCM_LIBRARIES})
