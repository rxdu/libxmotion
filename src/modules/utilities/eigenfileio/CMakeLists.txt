cmake_minimum_required(VERSION 3.0.0)
project(eigen_fileio)

## Find dependencies
find_package(Boost REQUIRED system filesystem)
include_directories(${Boost_INCLUDE_DIRS})

# Add libraries
add_library(eigenfileio INTERFACE)
target_link_libraries(eigenfileio INTERFACE ${Boost_LIBRARIES})
target_include_directories(eigenfileio INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Add tests
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
