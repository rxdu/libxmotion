# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
set(QUADCON_LIB_SRC
    # state estimator
    estimator/att_estimator.cpp
    # data types
    data_types/quad_state.cpp
    # controller
    # deprecated due to long-time lack of maintenance
    #controller/att_euler_con.cpp
    #controller/pos_euler_con.cpp
    controller/att_quat_con.cpp
    controller/pos_quat_con.cpp
    # motion management
    motion_server/motion_server.cpp
    motion_server/quad_poly_traj_handler.cpp
    #motion_server/waypoint_path_handler.cpp
    # data transmitter
    data_trans/quad_data_transmitter.cpp
)
add_library(quad_ctrl STATIC ${QUADCON_LIB_SRC})
target_link_libraries(quad_ctrl quadflat common ${LCM_LIBRARIES})

# Add executables
#add_executable(quad_motion_client motion_server/motion_client.cpp)
#target_link_libraries(quad_motion_client ${LCM_LIBRARIES})

#add_executable(pos_step controller/tests/pos_step_response.cpp)
#target_link_libraries(pos_step ${LCM_LIBRARIES})
