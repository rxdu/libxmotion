find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Add libraries
add_library(map_processing
    src/pgm_map.cpp
    src/point_cloud_processor.cpp
    src/trajectory_loader.cpp
)
target_link_libraries(map_processing PUBLIC
    interface
    pnm
    math_utils
    logging
    cvdraw
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES})
target_compile_definitions(map_processing PUBLIC ENABLE_VISUAL)
target_include_directories(map_processing PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# Add executables
if (BUILD_TESTS)
  add_subdirectory(test)
endif ()