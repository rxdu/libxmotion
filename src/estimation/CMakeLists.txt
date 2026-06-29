find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Add libraries
add_library(estimation STATIC
    src/mekf6.cpp
    src/mekf9.cpp)
target_link_libraries(estimation PUBLIC
    xmotion::hal
    Eigen3::Eigen)
target_include_directories(estimation PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

if (BUILD_APPS)
  add_subdirectory(app)
endif ()

if (BUILD_TESTS)
  add_subdirectory(test)
endif ()

## installation configuration
install(TARGETS estimation
    EXPORT xmNablaTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})