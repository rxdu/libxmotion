add_library(simlite INTERFACE)
target_link_libraries(simlite INTERFACE odeint)
target_include_directories(simlite INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>)

add_subdirectory(test)