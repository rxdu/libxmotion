# Dependency libraries
find_package(GSL REQUIRED)
include_directories(${GSL_INCLUDE_DIRS})

# Add libraries
set(REACHABILITY_LIB_SRC
  # Monte-Carlo based occupancy
  src/monte_carlo_sim.cpp
  # Markov-model based occupancy
  src/markov_occupancy.cpp
)
add_library(reachability STATIC ${REACHABILITY_LIB_SRC})
target_link_libraries(reachability decomp markov model random utilities ascent ${GSL_LIBRARIES})
target_include_directories(reachability PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

if(BUILD_TESTS)
  add_subdirectory(tests)
endif()
