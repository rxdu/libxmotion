# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## Find Gurobi
find_package(GUROBI QUIET)
if (NOT GUROBI_FOUND)
  message("-- Gurobi not found. Trajectory generation will not be built. Make sure you have set the GUROBI_HOME variable.")
else()
  # message("-- Gurobi found")
  include_directories(${GUROBI_INCLUDE_DIRS})
endif()

# Add libraries
if (GUROBI_FOUND)
set(TRAJ_SRC     
    src/path_manager.cpp
    src/quad_polyopt.cpp
)
add_library(quad_traj STATIC ${TRAJ_SRC})
target_link_libraries(quad_traj polyopt common ${LCM_LIBRARIES})
target_include_directories(quad_traj PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)
endif()

# Add executables
#add_executable(quad_motion_client motion_server/motion_client.cpp)
#target_link_libraries(quad_motion_client ${LCM_LIBRARIES})

# add_subdirectory(tests)