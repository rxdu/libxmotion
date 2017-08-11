# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
set(QUADCON_LIB_SRC
    # state estimator
    #estimator/att_estimator.cpp
    estimator/rc_car_state.cpp
)
add_library(rccar_ctrl STATIC ${QUADCON_LIB_SRC})
target_link_libraries(rccar_ctrl common utility ${LCM_LIBRARIES})

# Add executables
#add_executable(quad_motion_client motion_server/motion_client.cpp)
#target_link_libraries(quad_motion_client ${LCM_LIBRARIES})

#add_executable(pos_step controller/tests/pos_step_response.cpp)
#target_link_libraries(pos_step ${LCM_LIBRARIES})
