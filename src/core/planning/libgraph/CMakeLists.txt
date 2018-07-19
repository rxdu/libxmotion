# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Add libraries
set(GRAPH_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/graph)
set(GRAPH_LIB_SRC	
	${GRAPH_SRC_DIR}/graph.hpp
	${GRAPH_SRC_DIR}/vertex.hpp
	${GRAPH_SRC_DIR}/edge.hpp
	${GRAPH_SRC_DIR}/details/graph_impl.hpp
	${GRAPH_SRC_DIR}/details/vertex_impl.hpp
	${GRAPH_SRC_DIR}/details/edge_impl.hpp
	${GRAPH_SRC_DIR}/details/helper_func.hpp
	${GRAPH_SRC_DIR}/details/priority_queue.hpp	
	${GRAPH_SRC_DIR}/algorithms/astar.hpp
)
add_library(graph INTERFACE)
target_sources(graph INTERFACE ${GRAPH_LIB_SRC})
target_compile_definitions(graph INTERFACE -DUSE_UNORDERED_MAP)
target_include_directories(graph INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<INSTALL_INTERFACE:include>)