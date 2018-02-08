# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(OpenCV 2.4 REQUIRED)

# Add libraries
set(MAP_LIB_SRC
  # 2D map
  src/image_utils.cpp
  # src/map_utils.cpp
  src/map2d.cpp
  # src/sgrid_builder.cpp
)
add_library(map STATIC ${MAP_LIB_SRC})
target_link_libraries(map geometry common ${LCM_LIBRARIES} ${OpenCV_LIBS})
target_include_directories(map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

add_subdirectory(tests)

