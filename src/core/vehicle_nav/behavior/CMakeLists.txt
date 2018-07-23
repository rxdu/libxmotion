# Dependency libraries
# find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(BEHAVIOR_LIB_SRC
  src/lane_marker.cpp
)
add_library(behavior STATIC ${BEHAVIOR_LIB_SRC})
target_link_libraries(behavior common)
target_include_directories(behavior PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

# add_subdirectory(tests)

