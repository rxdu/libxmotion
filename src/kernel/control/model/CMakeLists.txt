## Dependency libraries
# find_package(LIB_NAME REQUIRED)

## Add libraries
set(MODEL_LIB_SRC
	src/point_model.cpp
	src/bicycle_model.cpp
)
add_library(model STATIC ${MODEL_LIB_SRC})
target_link_libraries(model utilities ascent)
target_include_directories(model PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(tests)
endif()