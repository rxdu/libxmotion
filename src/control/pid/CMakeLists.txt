## Dependency libraries
find_package(Eigen3 REQUIRED NO_MODULE)

## Add libraries
add_library(pid
        src/pid_controller.cpp)
target_link_libraries(pid PUBLIC Eigen3::Eigen)
target_include_directories(pid PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE src)

# Add executables
if (BUILD_TESTS)
    add_subdirectory(test)
endif ()