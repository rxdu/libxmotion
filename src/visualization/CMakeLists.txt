## Dependency libraries
find_package(Eigen3 REQUIRED NO_MODULE)

## Visualization for the navigation-stack modules. This is the only
## first-party module that links the rendering engine: the algorithm
## libraries stay visualization-free (same ABI regardless of options).
add_library(viz STATIC
    src/geometry_draw.cpp
    src/lattice_draw.cpp
    src/lattice_viz.cpp)
target_link_libraries(viz PUBLIC
    cvdraw
    mppi
    geometry
    decomp
    sampling
    lattice
    Eigen3::Eigen)
target_include_directories(viz PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
add_library(xmotion::viz ALIAS viz)

install(TARGETS viz
    EXPORT xmNavigationTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if (BUILD_TESTS)
  add_subdirectory(test)
endif ()
