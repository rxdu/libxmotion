# Add libraries
#add_subdirectory(ipc)
add_subdirectory(event)
add_subdirectory(math_utils)

## Header-only motion-domain types/interfaces re-homed into xmNabla
## (trajectory types + controller interface). These left xmSigma in the
## single-target refactor and now live under xmnabla/.
add_library(xmnabla_common INTERFACE)
target_include_directories(xmnabla_common INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(xmnabla_common INTERFACE xmotion::xmSigma)
add_library(xmotion::xmnabla_common ALIAS xmnabla_common)

install(TARGETS xmnabla_common
    EXPORT xmNablaTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
