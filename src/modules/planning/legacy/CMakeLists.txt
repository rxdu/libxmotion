# Dependency libraries
find_package(OpenCV REQUIRED)
find_package(Octomap REQUIRED)

# Add libraries
set(MAP_LIB_SRC
  # 2D map
  src/image_utils.cpp
  # src/map_utils.cpp
  src/grid_map.cpp
  # src/sgrid_builder.cpp
  src/octomap_server.cpp
  src/cube_array.cpp
  # src/quad_tree.cpp
  src/square_grid.cpp
  src/graph_builder.cpp
  src/librav_math.cpp
)
add_library(legacy STATIC ${MAP_LIB_SRC})
target_link_libraries(legacy navtypes graph decomp datalink ${OpenCV_LIBS} ${OCTOMAP_LIBRARY})
target_include_directories(legacy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${OCTOMAP_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

if(BUILD_TESTS)
  add_subdirectory(tests)
endif()
