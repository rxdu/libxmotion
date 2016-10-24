# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../vrep_sim/vrep_lib/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../vrep_sim/vrep_lib/remoteApi)

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
set(QUADSIM_LIB_SRC
    quad_sim_client.cpp
    quad_sim_controller.cpp
)
add_library(quadsim STATIC ${QUADSIM_LIB_SRC})
if (ENABLE_G3LOG)
	add_definitions(-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -DENABLE_LOG)
	target_link_libraries(quadsim vrep_sim quad_ctrl ctrl_utils ${LCM_LIBRARIES})
else (ENABLE_G3LOG)
	add_definitions(-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255)
	target_link_libraries(quadsim vrep_sim quad_ctrl ${LCM_LIBRARIES})
endif (ENABLE_G3LOG)

# Add executables
#add_executable(quadsim_demo quadsim_demo.cpp)
#target_link_libraries(quadsim_demo quadsim quad_ctrl)
