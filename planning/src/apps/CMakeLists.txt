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
#set(APP_LIB_SRC
#    examples.cpp
#)
#add_library(apps STATIC ${APP_LIB_SRC})
#target_link_libraries(apps ${OpenCV_LIBS})

# Add executables
add_executable(graph_example graph_example.cpp)
target_link_libraries(graph_example vis ${OpenCV_LIBS})

add_executable(simple_graph simple_graph.cpp)
#target_link_libraries(simple_graph )

add_executable(test_qtree test_quadtree.cpp)
target_link_libraries(test_qtree qtree vis map ${OpenCV_LIBS})

add_executable(test_sgrid test_squaregrid.cpp)
target_link_libraries(test_sgrid squaregrid vis map ${OpenCV_LIBS})

add_executable(test_image test_image.cpp)
target_link_libraries(test_image vis ${OpenCV_LIBS})

add_executable(test_traj test_traj.cpp)
target_link_libraries(test_traj map vis ${OpenCV_LIBS})

