## Dependency libraries
find_package(OpenCV REQUIRED)

## Add libraries
add_library(cvdraw src/cv_colors.cpp
    src/color_maps.cpp
    src/cv_canvas.cpp
    src/cv_io.cpp)
target_link_libraries(cvdraw ${OpenCV_LIBS})
target_include_directories(cvdraw PUBLIC	
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
if(BUILD_TESTS)
    add_subdirectory(test)
endif()

install(TARGETS cvdraw
		EXPORT xmotionTargets
		LIBRARY DESTINATION lib
		ARCHIVE DESTINATION lib
		RUNTIME DESTINATION bin
		INCLUDES DESTINATION include)

install(DIRECTORY include/
		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})