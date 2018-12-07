## Dependency libraries
find_package(OpenCV REQUIRED)

# Add libraries
set(SIM_VIEWER_LIB_SRC
    src/traffic_viewer.cpp
)
add_library(sim_viewer STATIC ${SIM_VIEWER_LIB_SRC})
target_link_libraries(sim_viewer trafficsim datalink traffic lightview lightviz ugvnav_viz utils ${OpenCV_LIBS})
target_include_directories(sim_viewer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(tests)
endif()
