## Dependency libraries
find_package(OpenCV REQUIRED)

# Add libraries
set(LIGHTVIEW_LIB_SRC
    src/viewer_utils.cpp
    src/lightwidgets.cpp
    src/lightviewer.cpp
)
add_library(lightview STATIC ${LIGHTVIEW_LIB_SRC})
target_link_libraries(lightview common utils imgui datalink ${OpenCV_LIBS})
target_include_directories(lightview PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

set(SIM_VIEWER_LIB_SRC
  src/traffic_viewer.cpp
)
add_library(trafficsim_viewer STATIC ${SIM_VIEWER_LIB_SRC})
target_link_libraries(trafficsim_viewer trafficsim datalink traffic lightview lightviz navviz utils ${OpenCV_LIBS})
target_include_directories(trafficsim_viewer PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
PRIVATE src)

add_subdirectory(apps)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(tests)
endif()
