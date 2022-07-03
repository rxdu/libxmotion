## Dependency libraries
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

## Add libraries
# add_library(model INTERFACE)
# target_link_libraries(model INTERFACE utilities ${Boost_LIBRARIES})
# target_include_directories(model INTERFACE
# 	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# 	$<INSTALL_INTERFACE:include>)

add_library(control src/pid_controller.cpp)
target_link_libraries(control PUBLIC utilities Eigen3::Eigen ${Boost_LIBRARIES})
target_include_directories(control PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

if(BUILD_TESTS)
    add_subdirectory(test)
endif()

## installation configuration
#install(TARGETS control
#        EXPORT roboswTargets
#        LIBRARY DESTINATION lib
#        ARCHIVE DESTINATION lib
#        RUNTIME DESTINATION bin
#        INCLUDES DESTINATION include)
#
#install(DIRECTORY include/
#        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/control)
