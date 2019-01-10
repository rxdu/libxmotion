# Dependency libraries
find_package(Eigen3 REQUIRED)

# Add libraries
set(UTILITY_LIB_SRC
	image_io/image_io.cpp
)
add_library(utils STATIC ${UTILITY_LIB_SRC})
target_link_libraries(utils eigenfileio)
target_compile_options(utils PUBLIC "-Wno-format-truncation")
target_include_directories(utils PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include>)

if(BUILD_TESTS)
	add_subdirectory(tests)
endif()