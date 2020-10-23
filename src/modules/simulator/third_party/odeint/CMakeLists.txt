# Dependency libraries
# find_package(Eigen3 REQUIRED)

# Add libraries
add_library(odeint INTERFACE)
target_include_directories(odeint INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
