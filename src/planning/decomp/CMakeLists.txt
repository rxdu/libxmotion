# Dependency libraries
# find_package(Eigen3 REQUIRED)

# Add libraries
add_library(decomp INTERFACE)
target_link_libraries(decomp INTERFACE geometry graph)
target_compile_definitions(decomp INTERFACE ENABLE_VISUAL)
target_include_directories(decomp INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)

# Add tests
if(BUILD_TESTS)
  add_subdirectory(test)
endif()
