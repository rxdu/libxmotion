# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Add executables
add_executable(test_gridmap test_gridmap.cpp)
target_link_libraries(test_gridmap map fastplot)

# add_executable(test_qtree test_quadtree.cpp)
# target_link_libraries(test_qtree geometry vis map)

# add_executable(test_sgrid test_squaregrid.cpp)
# target_link_libraries(test_sgrid geometry vis map)

# add_executable(test_sgridv2 test_squaregridv2.cpp)
# target_link_libraries(test_sgridv2 geometry vis map)

# add_executable(test_graphvis test_graphvis.cpp)
# target_link_libraries(test_graphvis geometry vis map)

# add_executable(test_image test_image.cpp)
# target_link_libraries(test_image map vis)

# add_executable(test_expansion test_expansion.cpp)
# target_link_libraries(test_expansion map vis)

# add_executable(test_traj test_traj.cpp)
# target_link_libraries(test_traj map vis)

# add_executable(octree_example octomap_example.cpp)
# target_link_libraries(octree_example ${Octomap_LIBS})

#add_executable(test_octomap tests/test_octomap.cpp)
#target_link_libraries(test_octomap ${Octomap_LIBS})

#add_executable(test_cubebuilder tests/test_cubebuilder.cpp)
#target_link_libraries(test_cubebuilder map geometry ${LCM_LIBRARIES})
