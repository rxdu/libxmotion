# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
set(QUADSIM_LIB_SRC
    rc_car_sim_client.cpp
    rc_car_sim_control.cpp
)
add_library(rccarsim STATIC ${QUADSIM_LIB_SRC})
add_definitions(-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255)
target_link_libraries(rccarsim vrep_sim common ${LCM_LIBRARIES})

# Add executables
#add_executable(quadsim_demo quadsim_demo.cpp)
#target_link_libraries(quadsim_demo quadsim quad_ctrl)
