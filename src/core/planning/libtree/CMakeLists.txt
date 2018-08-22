# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add libraries
add_library(tree INTERFACE)
# target_sources(tree INTERFACE 
# 	${CMAKE_CURRENT_SOURCE_DIR}/graph/graph.hpp
# 	)
# target_compile_definitions(tree INTERFACE -DUSE_UNORDERED_MAP -DMINIMAL_PRINTOUT)
target_include_directories(tree INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<INSTALL_INTERFACE:include>)