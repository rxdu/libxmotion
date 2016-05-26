# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/src/vrep_sim/vrep_lib/include)
include_directories(${PROJECT_SOURCE_DIR}/src/vrep_sim/vrep_lib/remoteApi)

# Add executables
add_executable(quad_vrepsim_con quad_vrepsim_con.cpp)
if (ENABLE_G3LOG)
	target_link_libraries(quad_vrepsim_con motion_server vrep_sim quad_con ctrl_utils g3logger)
	add_definitions(-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -DENABLE_LOG)
else (ENABLE_G3LOG)
	target_link_libraries(quad_vrepsim_con motion_server vrep_sim quad_con)
	add_definitions(-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255)
endif (ENABLE_G3LOG)
