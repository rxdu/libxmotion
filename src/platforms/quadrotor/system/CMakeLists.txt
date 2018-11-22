# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## Find Gurobi
find_package(GUROBI QUIET)
if (NOT GUROBI_FOUND)
  message("-- Gurobi not found. Trajectory generation will not be built. Make sure you have set the GUROBI_HOME variable")
else()
  # message("-- Gurobi found")
endif()

# Add libraries
set(QUAD_BASE_SRC
    src/quad_data_broadcaster.cpp    
    src/motion_server.cpp
    src/quad_flattraj_handler.cpp
    src/mission_tracker.cpp
)
add_library(quad_sys STATIC ${QUAD_BASE_SRC})
if (NOT GUROBI_FOUND)
  target_link_libraries(quad_sys quad_ctrl legacy common  datalink)
else()
  target_link_libraries(quad_sys quad_ctrl quad_traj legacy common  datalink)
endif()
target_include_directories(quad_sys PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

# Add executables
#add_executable(quad_motion_client motion_server/motion_client.cpp)
#target_link_libraries(quad_motion_client datalink)
