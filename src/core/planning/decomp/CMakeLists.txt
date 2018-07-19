# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
# find_package(LIBRARY_NAME REQUIRED)

# Add libraries
add_library(decomp INTERFACE)
target_include_directories(decomp INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Add tests
if(BUILD_TESTS)
  add_subdirectory(tests)
endif()
