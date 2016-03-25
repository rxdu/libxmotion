# Put all binary files into /bin and libraries into /lib
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_SOURCE_DIR}/src/map)
include_directories(${CMAKE_SOURCE_DIR}/src/quadtree)
include_directories(${CMAKE_SOURCE_DIR}/src/graph)
include_directories(${CMAKE_SOURCE_DIR}/src/visualizer)
include_directories(${CMAKE_SOURCE_DIR}/src/square_grid)

# Dependency libraries
find_package(OpenCV REQUIRED)

# Add libraries
#set(VIS_LIB_SRC
#    graph_vis.cpp
#)
#add_library(graph_vis STATIC ${VIS_LIB_SRC})
#target_link_libraries(vis ${OpenCV_LIBS})

# Add executables
add_executable(test_graph test_graph.cpp)
target_link_libraries(test_graph qtree vis map ${OpenCV_LIBS})

add_executable(test_grid test_squaregrid.cpp)
target_link_libraries(test_grid squaregrid vis map ${OpenCV_LIBS})

add_executable(test_image test_image.cpp)
target_link_libraries(test_image vis ${OpenCV_LIBS})

add_executable(test_plan test_plan.cpp)
target_link_libraries(test_plan vis ${OpenCV_LIBS})
