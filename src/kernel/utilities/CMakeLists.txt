# Dependency libraries
find_package(Threads REQUIRED)

## Find Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Add libraries
set(UTILITY_LIB_SRC
	logging/logger.cpp
	image_io/image_io.cpp
)
add_library(utilities STATIC ${UTILITY_LIB_SRC})
if(ENABLE_LOGGING)
target_compile_definitions(utilities PUBLIC -DENABLE_LOGGING)
endif(ENABLE_LOGGING)
target_link_libraries(utilities spdlog eigenfileio ${CMAKE_THREAD_LIBS_INIT})
target_compile_options(utilities PUBLIC "-Wno-format-truncation")
target_include_directories(utilities PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<INSTALL_INTERFACE:include>)

if(BUILD_TESTS)
	add_subdirectory(tests)
endif()