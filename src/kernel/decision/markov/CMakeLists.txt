## Dependency libraries
# find_package(LIB_NAME REQUIRED)

## Add libraries
set(MARKOV_LIB_SRC
	src/markov_chain.cpp
)
add_library(markov STATIC ${MARKOV_LIB_SRC})
target_link_libraries(markov utilities ascent)
target_include_directories(markov PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(tests)
endif()