if(BUILD_TESTS)
    add_subdirectory(googletest)
endif()

add_subdirectory(spdlog)
add_subdirectory(rapidcsv)

add_library(stopwatch INTERFACE)
target_include_directories(stopwatch INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/stopwatch>
    $<INSTALL_INTERFACE:include>)

add_subdirectory(spatial)

if(BUILD_TESTS)
    option(LCM_ENABLE_TESTS "Build LCM tests" ON)
else ()
    option(LCM_ENABLE_TESTS "Build LCM tests" OFF)
endif()
add_subdirectory(lcm)
