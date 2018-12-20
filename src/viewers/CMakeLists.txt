## Dependency libraries
find_package(OpenCV REQUIRED)

# Add libraries
set(VIEWER_LIB_SRC
    src/pose_viewer.cpp
)
add_library(viewers STATIC ${VIEWER_LIB_SRC})
target_link_libraries(viewers datalink lightview lightviz utils ${OpenCV_LIBS})
target_include_directories(viewers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(tests)
endif()
