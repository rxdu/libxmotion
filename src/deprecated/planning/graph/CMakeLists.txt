# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(GRAPH_LIB_SRC
	graph.h
	vertex.h
	edge.h
	bds_base.h
	bds_example.h
)
add_library(graph INTERFACE)
target_sources(graph INTERFACE ${GRAPH_LIB_SRC})
target_include_directories(graph PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<INSTALL_INTERFACE:include>)