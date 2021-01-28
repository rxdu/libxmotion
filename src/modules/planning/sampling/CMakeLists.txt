# Dependency libraries
find_package(Eigen3 REQUIRED NO_MODULE)

# Add libraries
set(SAMPLING_LIB_SRC
    src/rand_num_gen.cpp
)
add_library(sampling ${SAMPLING_LIB_SRC})
target_link_libraries(sampling graph geometry spatial cvdraw Eigen3::Eigen)
target_include_directories(sampling PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
  private src)

# if(BUILD_TESTS)
  add_subdirectory(tests)
# endif()
