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
add_executable(test_graph test_graph.cpp)
target_link_libraries(test_graph qtree vis map ${OpenCV_LIBS})

add_executable(test_grid test_squaregrid.cpp)
target_link_libraries(test_grid squaregrid vis map ${OpenCV_LIBS})

add_executable(test_image test_image.cpp)
target_link_libraries(test_image vis ${OpenCV_LIBS})

add_executable(run_example example.cpp)
target_link_libraries(run_example vis ${OpenCV_LIBS})

add_executable(test_traj test_traj.cpp)
target_link_libraries(test_traj map vis ${OpenCV_LIBS})

add_executable(simple_graph simple_graph.cpp)
#target_link_libraries(simple_graph )

