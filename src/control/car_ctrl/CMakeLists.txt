# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
set(CARCON_LIB_SRC    
    # state estimation
    estimator/car_state.cpp
)
add_library(car_ctrl STATIC ${CARCON_LIB_SRC})
target_link_libraries(car_ctrl common utility ${LCM_LIBRARIES})

# Add executables
#add_executable(quad_motion_client motion_server/motion_client.cpp)
#target_link_libraries(quad_motion_client ${LCM_LIBRARIES})
