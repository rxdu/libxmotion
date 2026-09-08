find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# SYSTEM: PCL's include dirs pull in the whole dependency prefix (Homebrew's
# /opt/homebrew/include, say). Without SYSTEM those precede this project's
# own -I paths and can shadow the vendored googletest headers, which then
# mismatch the vendored libgtest at link time. SYSTEM also keeps PCL's
# warnings from gating, matching how the other vendored deps are treated.
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Add libraries
add_library(map_processing
    src/pgm_map.cpp
    src/point_cloud_processor.cpp
    src/trajectory_processor.cpp
)
target_link_libraries(map_processing PUBLIC
    xmotion::xmBase
    xmotion::xmBaseGeometry
    xmotion::navtypes
    Eigen3::Eigen
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES})
target_link_libraries(map_processing PRIVATE pnm rapidcsv stb)
target_include_directories(map_processing PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# Add executables
if (BUILD_TESTS)
  add_subdirectory(test)
endif ()

install(TARGETS map_processing
    EXPORT xmNavigationTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
