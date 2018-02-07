# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Add libraries
set(SIM_INTERFACE_LIB_SRC
  # vrep remote api
  vrep_interface/vrep_api/remoteApi/extApi.c
  vrep_interface/vrep_api/remoteApi/extApiPlatform.c
)

add_library(sim_interface STATIC ${SIM_INTERFACE_LIB_SRC})
# add_definitions(-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255)
target_compile_definitions(sim_interface PUBLIC NON_MATLAB_PARSING MAX_EXT_API_CONNECTIONS=255)
if (UNIX)
 	target_link_libraries(sim_interface pthread)
endif (UNIX)
target_include_directories(sim_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>)

