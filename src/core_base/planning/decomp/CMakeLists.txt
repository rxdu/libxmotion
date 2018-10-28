# Dependency libraries
# find_package(LIBRARY_NAME REQUIRED)

set(ENABLE_CURVILINEAR_GRID on)

# Add libraries
add_library(decomp INTERFACE)
if(ENABLE_CURVILINEAR_GRID)
target_link_libraries(decomp INTERFACE geometry)
endif()
target_include_directories(decomp INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Add tests
if(BUILD_TESTS)
  add_subdirectory(tests)
endif()
