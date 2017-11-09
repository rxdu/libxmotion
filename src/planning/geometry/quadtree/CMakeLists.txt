# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(QTREE_LIB_SRC
    quad_tree.cpp
)
add_library(qtree STATIC ${QTREE_LIB_SRC})
#target_link_libraries(qtree ${OpenCV_LIBS})

# Add executables
#add_executable(build_quadtree test_quadtree.cpp)
#target_link_libraries(build_quadtree qtree graph)
