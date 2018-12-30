# Dependency libraries
find_package(Threads REQUIRED)

## Find Eigen3
find_package(Eigen3 REQUIRED)

# Add libraries
set(UTILITY_LIB_SRC
	# logging/logger.cpp
	image_io/image_io.cpp
)
add_library(utils STATIC ${UTILITY_LIB_SRC})
if(ENABLE_LOGGING)
target_compile_definitions(utils PUBLIC -DENABLE_LOGGING)
endif(ENABLE_LOGGING)
target_link_libraries(utils spdlog eigenfileio ${CMAKE_THREAD_LIBS_INIT})
target_compile_options(utils PUBLIC "-Wno-format-truncation")
target_include_directories(utils PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include>)

if(BUILD_TESTS)
	add_subdirectory(tests)
endif()