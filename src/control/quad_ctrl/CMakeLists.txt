# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
set(QUADCON_LIB_SRC
    # state estimator
    estimator/att_estimator.cpp
    # data types
    state/quad_state.cpp
    # controller
    controller/att_quat_con.cpp
    controller/pos_quat_con.cpp
    # deprecated due to lack of maintenance
    #controller/att_euler_con.cpp
    #controller/pos_euler_con.cpp    
    # actuator
    actuator/quad_mixer.cpp    
)
add_library(quad_ctrl STATIC ${QUADCON_LIB_SRC})
target_link_libraries(quad_ctrl common utility ${LCM_LIBRARIES})

# Add executables
#add_executable(quad_motion_client motion_server/motion_client.cpp)
#target_link_libraries(quad_motion_client ${LCM_LIBRARIES})
