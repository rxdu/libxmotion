## Dependency libraries
## Find Eigen3
find_package(Eigen3 REQUIRED)

## Add libraries
set(MARKOV_LIB_SRC
	src/markov_chain.cpp
)
add_library(markov STATIC ${MARKOV_LIB_SRC})
target_link_libraries(markov utils odeint)
target_include_directories(markov PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
if(BUILD_TESTING)
	add_subdirectory(tests)
endif()