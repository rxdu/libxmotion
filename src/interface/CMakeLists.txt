find_package(Eigen3 REQUIRED NO_MODULE)

add_library(interface INTERFACE)
target_link_libraries(interface INTERFACE Eigen3::Eigen)
target_include_directories(interface INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

## installation configuration
install(TARGETS interface
    EXPORT xmotionTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
