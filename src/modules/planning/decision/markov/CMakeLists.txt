## Dependency libraries
## Find Eigen3
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

## Add libraries
add_library(markov src/markov_chain.cpp)
target_link_libraries(markov utils ${Boost_LIBRARIES})
target_include_directories(markov PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(tests)
endif()