add_subdirectory(spdlog)
add_subdirectory(yaml-cpp)

add_library(csv_parser INTERFACE)
target_include_directories(csv_parser INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/csv_parser>
    $<INSTALL_INTERFACE:include>)
    
add_library(stopwatch INTERFACE)
target_include_directories(stopwatch INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/stopwatch>
    $<INSTALL_INTERFACE:include>)
