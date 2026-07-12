# Header-only navigation-stack vocabulary (types + interfaces)

## Header-only motion-domain types/interfaces re-homed into xmNavigation
## (trajectory types + controller interface). These left xmBase in the
## single-target refactor and now live under xmnavigation/.
add_library(navtypes INTERFACE)
target_include_directories(navtypes INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(navtypes INTERFACE xmotion::xmBase xmotion::xmBaseGeometry)
add_library(xmotion::navtypes ALIAS navtypes)

install(TARGETS navtypes
    EXPORT xmNavigationTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
