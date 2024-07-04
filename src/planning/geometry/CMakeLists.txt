## Dependency libraries
find_package(Eigen3 REQUIRED NO_MODULE)

## Add libraries
add_library(geometry STATIC
    src/polyline.cpp
    src/cubic_spline.cpp
    src/parametric_curve.cpp
    src/polygon.cpp
)
target_link_libraries(geometry PUBLIC Eigen3::Eigen)
if (ENABLE_TARGET_DESKTOP)
  target_compile_definitions(geometry PUBLIC ENABLE_VISUAL)
  target_link_libraries(geometry PUBLIC cvdraw)
endif ()
target_include_directories(geometry PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# Add executables
if (BUILD_TESTS)
  add_subdirectory(test)
endif ()