# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/vrep_lib/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/vrep_lib/remoteApi)

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
set(VREPSIM_LIB_SRC
  # vrep remote api
  vrep_lib/remoteApi/extApi.c
  vrep_lib/remoteApi/extApiPlatform.c
  # vrep quad simulation
  #vrep_client/quad_sim_client.cpp
  #sim_process/quad_sim_process.cpp
)

add_library(vrep_sim STATIC ${VREPSIM_LIB_SRC})
if (ENABLE_G3LOG)
	add_definitions(-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -DENABLE_LOG)
else (ENABLE_G3LOG)
	add_definitions(-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255)
endif (ENABLE_G3LOG)

if (UNIX)
 	target_link_libraries(vrep_sim pthread)
endif (UNIX)

# Add executables
#add_executable(quad_vrep_sim main.cpp)
#if (ENABLE_G3LOG)
#	target_link_libraries(quad_vrep_sim vrep_sim quad_con g3logger)
#else (ENABLE_G3LOG)
#	target_link_libraries(quad_vrep_sim vrep_sim quad_con)
#endif (ENABLE_G3LOG)
