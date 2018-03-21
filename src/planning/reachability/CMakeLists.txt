# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(GSL REQUIRED)
include_directories(${GSL_INCLUDE_DIRS})

# Add libraries
set(REACHABILITY_LIB_SRC
  src/vehicle_dynamics.cpp
  src/bigaussian_sampler.cpp
  src/monte_carlo_sim.cpp
)
add_library(reachability STATIC ${REACHABILITY_LIB_SRC})
target_link_libraries(reachability utilities ascent ${GSL_LIBRARIES})
target_include_directories(reachability PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

add_subdirectory(tests)

