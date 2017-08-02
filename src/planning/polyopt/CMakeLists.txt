# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

## Conditional add sources
find_package(GUROBI)
if (NOT GUROBI_FOUND)
  message("Gurobi not found. If you want to use Gurobi, set the GUROBI_HOME variable")
endif()

message("Gurobi found")
include_directories(${GUROBI_INCLUDE_DIRS})

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
    #poly_curve.cpp
    gurobi_utils.cpp
    gurobi_polyopt.cpp
)
add_library(polyopt STATIC ${POLYOPT_LIB_SRC})
target_link_libraries(polyopt common ${GUROBI_LIBRARIES})

# Add executables
#add_executable(qp_cpp tests/qp_c++.cpp)
#target_link_libraries(qp_cpp ${GUROBI_LIBRARIES})

#add_executable(test_utils tests/test_utils.cpp)
#target_link_libraries(test_utils polyopt ${GUROBI_LIBRARIES})

#add_executable(test_qp tests/test_qp.cpp)
#target_link_libraries(test_qp polyopt vis ${GUROBI_LIBRARIES})

#add_executable(test_qp2 tests/test_qp2.cpp)
#target_link_libraries(test_qp2 polyopt ${GUROBI_LIBRARIES})

#add_executable(test_qp3 tests/test_qp3.cpp)
#target_link_libraries(test_qp3 polyopt ${GUROBI_LIBRARIES})

#add_executable(test_opt tests/test_opt.cpp)
#target_link_libraries(test_opt polyopt ${GUROBI_LIBRARIES})

#add_executable(test_corridor tests/test_corridor.cpp)
#target_link_libraries(test_corridor polyopt ${GUROBI_LIBRARIES} ${LCM_LIBRARIES})
