# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add libraries
add_library(tree INTERFACE)
target_sources(tree INTERFACE 
	${CMAKE_CURRENT_SOURCE_DIR}/include/tree/tree_node.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/tree/basic_tree.hpp
)
# target_compile_definitions(tree INTERFACE -DUSE_UNORDERED_MAP -DMINIMAL_PRINTOUT)
target_include_directories(tree INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>)

if(BUILD_TESTS)
	add_subdirectory(tests)
endif()
  