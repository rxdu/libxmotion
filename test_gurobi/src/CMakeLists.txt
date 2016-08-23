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
#set(LIBHELLO_SRC hello.cpp)
#add_library(hello STATIC ${LIBHELLO_SRC})

# Add executables
add_executable(test_qp qp_c++.cpp)
target_link_libraries(test_qp ${GUROBI_LIBRARIES})

add_executable(qp_traj qp_traj.cpp)
target_link_libraries(qp_traj ${GUROBI_LIBRARIES})
