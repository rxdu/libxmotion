# Dependency libraries
find_package(OpenCV REQUIRED)

find_package(Octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

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
)
add_library(map STATIC ${MAP_LIB_SRC})
target_link_libraries(map common ${LCM_LIBRARIES} ${OpenCV_LIBS} ${OCTOMAP_LIBRARY})
target_include_directories(map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

if(BUILD_TESTS)
  add_subdirectory(tests)
endif()
