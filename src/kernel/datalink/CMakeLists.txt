cmake_minimum_required(VERSION 3.0.0)
project(librav_datalink)

## Dependency libraries
## Find LCM
find_package(LCM REQUIRED)
if(LCM_NOT_FOUND)
  message(STATUS "LCM is not found. Inter-process communication depends on LCM.")
endif(LCM_NOT_FOUND)

## Add libraries
add_library(datalink INTERFACE)
target_link_libraries(datalink INTERFACE ${LCM_LIBRARIES})
target_include_directories(datalink INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
	$<INSTALL_INTERFACE:include>)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(tests)
endif()
