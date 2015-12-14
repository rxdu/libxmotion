# Put all binary files into /bin and libraries into /lib
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(OpenCV REQUIRED)

# Add libraries
set(QTREE_LIB_SRC
    quad_tree.cpp
	qtree_builder.cpp
)
add_library(qtree STATIC ${QTREE_LIB_SRC})
target_link_libraries(qtree ${OpenCV_LIBS})

# Add executables
add_executable(build_quadtree test_quadtree.cpp)
target_link_libraries(build_quadtree qtree ${OpenCV_LIBS})
