## Dependency libraries
find_package(Eigen3 REQUIRED NO_MODULE)

## MPPI is a header-only templated controller (model/cost/sampler seams
## resolve at compile time so the rollout loop fully inlines).
add_library(mppi INTERFACE)
target_link_libraries(mppi INTERFACE Eigen3::Eigen)
target_include_directories(mppi INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

if (BUILD_TESTS)
  add_subdirectory(test)
endif ()

install(TARGETS mppi
    EXPORT xmNavigationTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
