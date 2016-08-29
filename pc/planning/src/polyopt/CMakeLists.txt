# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

find_package(GUROBI REQUIRED)
if (NOT GUROBI_FOUND)
  message("Gurobi not found. If you want to use Gurobi, set the GUROBI_HOME variable")
else()
  message("Gurobi found")
  include_directories(${GUROBI_INCLUDE_DIRS})
endif()

# Add libraries
set(POLYOPT_LIB_SRC 
    polyopt_math.cpp
    gurobi_utils.cpp
)
add_library(polyopt STATIC ${POLYOPT_LIB_SRC})

# Add executables
add_executable(qp_cpp qp_c++.cpp)
target_link_libraries(qp_cpp ${GUROBI_LIBRARIES})

add_executable(qp_traj qp_traj.cpp)
target_link_libraries(qp_traj ${GUROBI_LIBRARIES})

add_executable(test_utils test_utils.cpp)
target_link_libraries(test_utils polyopt ${GUROBI_LIBRARIES})

add_executable(test_qp test_qp.cpp)
target_link_libraries(test_qp polyopt vis ${GUROBI_LIBRARIES})

add_executable(test_qp2 test_qp2.cpp)
target_link_libraries(test_qp2 polyopt ${GUROBI_LIBRARIES})