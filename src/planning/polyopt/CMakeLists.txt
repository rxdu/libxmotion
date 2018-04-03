# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

## Find Gurobi
find_package(GUROBI QUIET)
if (NOT GUROBI_FOUND)
  message("-- Gurobi not found. Trajectory optimization will not be built.")
else()
  message("-- Gurobi found")
  include_directories(${GUROBI_INCLUDE_DIRS})
endif()

# Add libraries
if(GUROBI_FOUND)
set(POLYOPT_LIB_SRC
    src/polyopt_math.cpp
    src/gurobi_solver/gurobi_utils.cpp
    src/gurobi_solver/gurobi_polyopt.cpp
)
add_library(polyopt STATIC ${POLYOPT_LIB_SRC})
target_link_libraries(polyopt common gurobi_cpp ${GUROBI_LIBRARIES})
target_include_directories(polyopt PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)
endif()

# Add executables
#add_subdirectory(tests)