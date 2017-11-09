# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/src)

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add executables
## applications
#add_executable(quadsim_planner quadsim_planner.cpp)
#target_link_libraries(quadsim_planner map planner ${LCM_LIBRARIES})

## tests
add_executable(graph_example graph_example.cpp)
target_link_libraries(graph_example vis map geometry)

add_executable(simple_graph simple_graph.cpp)
#target_link_libraries(simple_graph )
