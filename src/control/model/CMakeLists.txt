## Dependency libraries
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

## Add libraries
 add_library(model INTERFACE)
 target_link_libraries(model INTERFACE ${Boost_LIBRARIES})
 target_include_directories(model INTERFACE
 	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 	$<INSTALL_INTERFACE:include>)

if (BUILD_TESTS)
  add_subdirectory(test)
endif ()

## installation configuration
install(TARGETS model
    EXPORT xmotionTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
