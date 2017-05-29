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
set(QUADFLAT_LIB_SRC
    quad_flattraj.cpp
    quad_polyopt.cpp
)
add_library(quadflat STATIC ${QUADFLAT_LIB_SRC})
target_link_libraries(quadflat polyopt ${GUROBI_LIBRARIES})

# Add executables
#add_executable(test_quadopt tests/test_quadopt.cpp)
#target_link_libraries(test_quadopt polyopt quadflat ${GUROBI_LIBRARIES} ${LCM_LIBRARIES})

#add_executable(test_jointopt tests/test_jointopt.cpp)
#target_link_libraries(test_jointopt polyopt quadflat ${GUROBI_LIBRARIES} ${LCM_LIBRARIES})

#add_executable(test_jointcorridor tests/test_jointcorridor.cpp)
#target_link_libraries(test_jointcorridor polyopt quadflat ${GUROBI_LIBRARIES} ${LCM_LIBRARIES})

add_executable(test_trajdef tests/test_trajdef.cpp)
target_link_libraries(test_trajdef polyopt quadflat ${GUROBI_LIBRARIES})
