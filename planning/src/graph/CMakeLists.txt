# Put all binary files into /bin and libraries into /lib
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(OpenCV REQUIRED)

# Add libraries
set(GRAPH_LIB_SRC
	graph.cpp
)
add_library(graph STATIC ${GRAPH_LIB_SRC})
target_link_libraries(graph ${OpenCV_LIBS})

# Add executables
add_executable(build_graph test_graph.cpp)
target_link_libraries(build_graph graph ${OpenCV_LIBS})
