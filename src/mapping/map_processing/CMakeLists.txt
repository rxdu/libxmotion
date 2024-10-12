find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Add libraries
add_library(map_processing
    src/pgm_map.cpp
    src/point_cloud_processor.cpp
    src/trajectory_processor.cpp
)
target_link_libraries(map_processing PUBLIC
    interface
    pnm
    rapidcsv
    math_utils
    logging
    Eigen3::Eigen
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