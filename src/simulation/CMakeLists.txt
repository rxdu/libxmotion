## Dependency libraries
find_package(Eigen3 REQUIRED NO_MODULE)

## Headless simulation support for algorithm development (deterministic
## fixed-step loop + recording). Header-only; the visual viewer lives in
## the visualization module and attaches through the observer callback.
add_library(sim INTERFACE)
target_link_libraries(sim INTERFACE Eigen3::Eigen)
target_include_directories(sim INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
add_library(xmotion::sim ALIAS sim)

if (BUILD_TESTS)
  add_subdirectory(test)
endif ()

if (ENABLE_VISUALIZATION)
  add_subdirectory(demos)
endif ()

install(TARGETS sim
    EXPORT xmNavigationTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
