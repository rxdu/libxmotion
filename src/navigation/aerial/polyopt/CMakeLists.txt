## Find Eigen3
find_package(Eigen3 REQUIRED)

## Find Gurobi
find_package(GUROBI QUIET)
if (NOT GUROBI_FOUND)
  message("-- Gurobi not found. Trajectory optimization will not be built.")
else()
  message("-- Gurobi found")
endif()

# Add libraries
if(GUROBI_FOUND)
set(POLYOPT_LIB_SRC
    src/polyopt_math.cpp
    src/poly_curve.cpp
    src/gurobi_solver/gurobi_utils.cpp
    src/gurobi_solver/gurobi_polyopt.cpp
    src/osqp_solver/osqp_polyopt.cpp
)
add_library(polyopt STATIC ${POLYOPT_LIB_SRC})
target_link_libraries(polyopt common gurobi_cpp ${GUROBI_LIBRARIES})
target_include_directories(polyopt PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${GUROBI_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)
endif()

# Add executables
if(BUILD_TESTS)
  add_subdirectory(tests)
endif()