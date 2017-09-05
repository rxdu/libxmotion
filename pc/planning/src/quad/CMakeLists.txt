# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
set(QUAD_LIB_SRC
    mission_tracker.cpp
    path_manager.cpp
    quad_polyopt.cpp
)
add_library(quad STATIC ${QUAD_LIB_SRC})
target_link_libraries(quad map local polyopt ${OpenCV_LIBS} ${Octomap_LIBS} ${LCM_LIBRARIES})

# Add executables
#add_executable(wp_manager tests/wp_manager.cpp)
#target_link_libraries(wp_manager quad ${LCM_LIBRARIES})

#add_executable(test_waypoints tests/test_wpmanager.cpp)
#target_link_libraries(test_waypoints quad map geometry local polyopt ${Octomap_LIBS} ${LCM_LIBRARIES})

#add_executable(test_quadopt tests/test_quadopt.cpp)
#target_link_libraries(test_quadopt polyopt ${GUROBI_LIBRARIES} ${LCM_LIBRARIES})

#add_executable(test_jointopt tests/test_jointopt.cpp)
#target_link_libraries(test_jointopt polyopt ${GUROBI_LIBRARIES} ${LCM_LIBRARIES})

#add_executable(test_jointcorridor tests/test_jointcorridor.cpp)
#target_link_libraries(test_jointcorridor polyopt ${GUROBI_LIBRARIES} ${LCM_LIBRARIES})

add_executable(test_trajdef tests/test_trajdef.cpp)
target_link_libraries(test_trajdef polyopt ${GUROBI_LIBRARIES})
