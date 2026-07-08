## Dependency libraries
find_package(Eigen3 REQUIRED NO_MODULE)

## Output-stage safety shield (docs/control/safety_shield.md): command
## envelope + CBF barrier filter (QuadProg++) + ctfsm fallback ladder.
## Header-only; algorithm-core composition (no runtime tier).
add_library(shield INTERFACE)
target_link_libraries(shield INTERFACE
    xmotion::xmBase
    ctfsm::ctfsm
    quadprogpp
    Eigen3::Eigen)
target_include_directories(shield INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
add_library(xmotion::shield ALIAS shield)

if (BUILD_TESTS)
  add_subdirectory(test)
endif ()

install(TARGETS shield
    EXPORT xmNavigationTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
