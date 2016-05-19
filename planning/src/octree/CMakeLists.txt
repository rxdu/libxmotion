# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(OpenCV REQUIRED)
#find_package(octomap REQUIRED)
#include_directories(${OCTOMAP_INCLUDE_DIRS})

#message(STATUS "Found Octomap library at: ${OCTOMAP_INCLUDE_DIRS}")

# Add libraries
#set(OCTREE_LIB_SRC
#    octomap_example.cpp
#)
#add_library(octree STATIC ${OCTREE_LIB_SRC})
#target_link_libraries(qtree ${OpenCV_LIBS})

# Add executables
add_executable(octree_example octomap_example.cpp)
target_link_libraries(octree_example ${OCTOMAP_LIBRARIES})
