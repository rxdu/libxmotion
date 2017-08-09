# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(OpenCV REQUIRED)

# Add libraries
set(MAP_LIB_SRC
  # 2D map
  image_utils.cpp
  map_utils.cpp
  # 3D map
  octomap_server.cpp
  transformation.cpp
)
add_library(map STATIC ${MAP_LIB_SRC})
target_link_libraries(map geometry ${Octomap_LIBS} ${LCM_LIBRARIES} ${OpenCV_LIBS})

# Add executables
add_executable(test_qtree tests/test_quadtree.cpp)
target_link_libraries(test_qtree geometry vis map ${OpenCV_LIBS})

add_executable(test_sgrid tests/test_squaregrid.cpp)
target_link_libraries(test_sgrid geometry vis map ${OpenCV_LIBS})

add_executable(test_sgridv2 tests/test_squaregridv2.cpp)
target_link_libraries(test_sgridv2 geometry vis map ${OpenCV_LIBS})

add_executable(test_graphvis tests/test_graphvis.cpp)
target_link_libraries(test_graphvis geometry vis map ${OpenCV_LIBS})

add_executable(test_image tests/test_image.cpp)
target_link_libraries(test_image map vis ${OpenCV_LIBS})

add_executable(test_expansion tests/test_expansion.cpp)
target_link_libraries(test_expansion map vis ${OpenCV_LIBS})

add_executable(test_traj tests/test_traj.cpp)
target_link_libraries(test_traj map vis ${OpenCV_LIBS})

add_executable(octree_example tests/octomap_example.cpp)
target_link_libraries(octree_example ${Octomap_LIBS})

#add_executable(test_octomap tests/test_octomap.cpp)
#target_link_libraries(test_octomap local ${Octomap_LIBS})

#add_executable(test_cubebuilder tests/test_cubebuilder.cpp)
#target_link_libraries(test_cubebuilder map geometry local ${Octomap_LIBS} ${LCM_LIBRARIES})
