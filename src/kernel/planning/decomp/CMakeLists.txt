# Dependency libraries
## Find Eigen3
find_package(Eigen3 REQUIRED)

set(ENABLE_CURVILINEAR_GRID on)

# Add libraries
add_library(decomp INTERFACE)
if(ENABLE_CURVILINEAR_GRID)
target_link_libraries(decomp INTERFACE geometry)
endif()
target_include_directories(decomp INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)

# Add tests
if(BUILD_TESTS)
  add_subdirectory(tests)
endif()
