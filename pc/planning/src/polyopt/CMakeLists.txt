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
	# optimization solver
    polyopt_math.cpp
    gurobi_utils.cpp
    traj_optimizer.cpp
    polytraj_curve.cpp
    # quadrotor
    #quad_flattraj.cpp
    #quad_polyopt.cpp
)
add_library(polyopt STATIC ${POLYOPT_LIB_SRC})
target_link_libraries(polyopt ${GUROBI_LIBRARIES})

# Add executables
add_executable(qp_cpp tests/qp_c++.cpp)
target_link_libraries(qp_cpp ${GUROBI_LIBRARIES})

add_executable(test_utils tests/test_utils.cpp)
target_link_libraries(test_utils polyopt ${GUROBI_LIBRARIES})

add_executable(test_qp tests/test_qp.cpp)
target_link_libraries(test_qp polyopt vis ${GUROBI_LIBRARIES})

add_executable(test_qp2 tests/test_qp2.cpp)
target_link_libraries(test_qp2 polyopt ${GUROBI_LIBRARIES})

add_executable(test_qp3 tests/test_qp3.cpp)
target_link_libraries(test_qp3 polyopt ${GUROBI_LIBRARIES})

add_executable(test_opt tests/test_opt.cpp)
target_link_libraries(test_opt polyopt ${GUROBI_LIBRARIES})

#add_executable(test_trajdef tests/test_trajdef.cpp)
#target_link_libraries(test_trajdef polyopt ${GUROBI_LIBRARIES})

#add_executable(test_quadopt test_quadopt.cpp)
#target_link_libraries(test_quadopt polyopt ${GUROBI_LIBRARIES})