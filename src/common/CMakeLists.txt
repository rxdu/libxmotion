# Add libraries
#add_subdirectory(ipc)
add_subdirectory(event)
add_subdirectory(math_utils)

## Header-only motion-domain types/interfaces re-homed into xmNavigation
## (trajectory types + controller interface). These left xmBase in the
## single-target refactor and now live under xmnavigation/.
add_library(xmnavigation_common INTERFACE)
target_include_directories(xmnavigation_common INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(xmnavigation_common INTERFACE xmotion::xmBase)
add_library(xmotion::xmnavigation_common ALIAS xmnavigation_common)

install(TARGETS xmnavigation_common
    EXPORT xmNavigationTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
