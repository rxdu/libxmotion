# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
# find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(RRT_LIB_SRC
  src/lane_marker.cpp
)
add_library(rrt STATIC ${RRT_LIB_SRC})
target_link_libraries(rrt common)
target_include_directories(rrt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

# add_subdirectory(tests)

