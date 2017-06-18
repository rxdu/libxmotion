# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../vrep_sim/vrep_lib/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../vrep_sim/vrep_lib/remoteApi)

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
set(QUADSIM_LIB_SRC
    quad_hbird_sim_client.cpp
    quad_hbird_sim_controller.cpp
)
add_library(quadsim STATIC ${QUADSIM_LIB_SRC})
add_definitions(-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255)
target_link_libraries(quadsim vrep_sim quad_ctrl common ${LCM_LIBRARIES})

# Add executables
#add_executable(quadsim_demo quadsim_demo.cpp)
#target_link_libraries(quadsim_demo quadsim quad_ctrl)
