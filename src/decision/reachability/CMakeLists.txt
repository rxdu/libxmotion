# Dependency libraries
find_package(Eigen3 REQUIRED NO_MODULE)

# Eigen Flags
# EIGEN_STACK_ALLOCATION_LIMIT=0 => no limit on stack size
#
# When you run the application, make sure you set statck limit as unlimited in the OS:
#   
# $ ulimit -s unlimited
#
# Reference: https://eigen.tuxfamily.org/dox/TopicPreprocessorDirectives.html

# Add libraries
set(REACHABILITY_LIB_SRC
    # random number sampler
    src/random/uniform_sampler.cpp
    src/random/gaussian_sampler.cpp
	src/random/bigaussian_sampler.cpp
    # Monte-Carlo based occupancy
    src/monte_carlo_sim.cpp
    # Markov-model based occupancy
    src/tstate_transition_sim.cpp
    src/car_longitudinal_model.cpp
)
add_library(reachability STATIC ${REACHABILITY_LIB_SRC})
target_link_libraries(reachability xmotion::xmBase markov model stopwatch Eigen3::Eigen)
# target_compile_definitions(reachability PUBLIC "-DEIGEN_STACK_ALLOCATION_LIMIT=0")
target_include_directories(reachability PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

# apps/ excluded: gen_tstate_transition depends on the retired traffic_map
# module; re-enable with the road-network revival.

if (BUILD_TESTS)
  add_subdirectory(tests)
endif ()

install(TARGETS reachability
    EXPORT xmNavigationTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
