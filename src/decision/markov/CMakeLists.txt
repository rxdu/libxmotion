## Dependency libraries
## Find Eigen3
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED)

## Add libraries
add_library(markov src/markov_chain.cpp)
if (TARGET Boost::headers)
  set(XMNAV_BOOST Boost::headers)
else ()
  set(XMNAV_BOOST Boost::boost)
endif ()
target_link_libraries(markov Eigen3::Eigen ${XMNAV_BOOST})
target_include_directories(markov PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(tests)
endif()

install(TARGETS markov
    EXPORT xmNavigationTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
