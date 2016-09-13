# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
set(QUADCON_LIB_SRC
    # state estimator
    estimator/att_estimator.cpp
    estimator/quad_state.cpp
    # controller
    controller/att_euler_con.cpp
    controller/pos_euler_con.cpp
    controller/att_quat_con.cpp
    controller/pos_quat_con.cpp
    # motion management
    motion_server/motion_server.cpp
    # data transmitter
    data_trans/quad_data_transmitter.cpp
)
add_library(quad_ctrl STATIC ${QUADCON_LIB_SRC})
if (ENABLE_G3LOG)
  add_definitions(-DENABLE_LOG)
  target_link_libraries(quad_ctrl ctrl_utils g3logger ${LCM_LIBRARIES})
else(ENABLE_G3LOG)
  target_link_libraries(quad_ctrl ${LCM_LIBRARIES})
endif (ENABLE_G3LOG)

# Add executables
add_executable(quad_motion_client motion_server/motion_client.cpp)
target_link_libraries(quad_motion_client ${LCM_LIBRARIES})
