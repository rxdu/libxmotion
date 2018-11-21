## Dependency libraries
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})

# Add libraries
set(LIGHTVIEW_LIB_SRC
    src/viewer_utils.cpp
    src/lightviewer.cpp
)
add_library(lightview STATIC ${LIGHTVIEW_LIB_SRC})
target_link_libraries(lightview common utilities imgui ${OpenCV_LIBS} ${LCM_LIBRARIES})
target_include_directories(lightview PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(tests)
endif()
