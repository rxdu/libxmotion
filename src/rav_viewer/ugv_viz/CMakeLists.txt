## Dependency libraries
find_package(OpenCV REQUIRED)

# Add libraries
set(UGVVIZ_LIB_SRC
    src/traffic_viewer.cpp
)
add_library(ugvviz STATIC ${UGVVIZ_LIB_SRC})
target_link_libraries(ugvviz common traffic lightview lightviz ugvnav_viz utils datalink ${OpenCV_LIBS})
target_include_directories(ugvviz PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(tests)
endif()
