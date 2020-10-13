## Add libraries
add_subdirectory(third_party)

set(VIEWER_LIB_SRC 
    src/lightwindow.cpp
    src/imgui_base.cpp
)
add_library(viewer STATIC ${VIEWER_LIB_SRC})
target_compile_definitions(viewer PRIVATE "GLFW_INCLUDE_NONE")
target_link_libraries(viewer imgui)
target_include_directories(viewer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>	
    PRIVATE src)

# if(BUILD_TESTS)
	add_subdirectory(tests)
# endif()
