# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(OpenCV REQUIRED)

# Add libraries
set(MAP_LIB_SRC
  # 2D map
  src/image_utils.cpp
  src/map_utils.cpp
  # 3D map
  src/octomap_server.cpp
)
add_library(map STATIC ${MAP_LIB_SRC})
target_link_libraries(map geometry common ${Octomap_LIBS} ${LCM_LIBRARIES} ${OpenCV_LIBS})
target_include_directories(map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

add_subdirectory(src/tests)

