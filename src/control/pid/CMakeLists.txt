## Dependency libraries
find_package(Eigen3 REQUIRED NO_MODULE)

## Classical linear feedback family: production PID (scalar), state
## feedback with optional integral action (multi-dimensional states),
## and DLQR gain synthesis. Header-only.
add_library(pid INTERFACE)
target_link_libraries(pid INTERFACE xmotion::xmBase Eigen3::Eigen)
target_include_directories(pid INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
add_library(xmotion::pid ALIAS pid)

if (BUILD_TESTS)
  add_subdirectory(test)
endif ()

install(TARGETS pid
    EXPORT xmNavigationTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
