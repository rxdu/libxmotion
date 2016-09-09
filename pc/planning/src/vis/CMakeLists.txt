# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/src)

# Dependency libraries
find_package(OpenCV REQUIRED)

# Add libraries
set(VIS_LIB_SRC
    graph_vis.cpp
    curve_vis.cpp
    rrt_vis.cpp
    vis_utils.cpp
)
add_library(vis STATIC ${VIS_LIB_SRC})
target_link_libraries(vis map geometry ${OpenCV_LIBS})

# Add executables
#add_executable(test_graph test_graph.cpp)
#target_link_libraries(test_graph qtree vis map ${OpenCV_LIBS})
