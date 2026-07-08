## Dependency libraries
find_package(Eigen3 REQUIRED NO_MODULE)

## Platform models shared across the stack: MPPI rollouts (CPU + CUDA via
## the raw-span cores), the simulation tier, and the reachability
## Monte-Carlo propagation all use the same Step-concept models.
add_library(models INTERFACE)
target_link_libraries(models INTERFACE Eigen3::Eigen)
target_include_directories(models INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
add_library(xmotion::models ALIAS models)

if (BUILD_TESTS)
  add_subdirectory(test)
endif ()

install(TARGETS models
    EXPORT xmNavigationTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
