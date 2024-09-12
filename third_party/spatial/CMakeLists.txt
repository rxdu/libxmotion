cmake_minimum_required(VERSION 3.5)
project(spatial)

add_library(spatial INTERFACE)
target_include_directories(spatial INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>)

install(TARGETS spatial
		EXPORT xmotionTargets
		LIBRARY DESTINATION lib
		ARCHIVE DESTINATION lib
		RUNTIME DESTINATION bin
		INCLUDES DESTINATION include)

install(DIRECTORY include/
		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
