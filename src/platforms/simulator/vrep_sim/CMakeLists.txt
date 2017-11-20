# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/vrep_lib/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/vrep_lib/remoteApi)

# Add libraries
set(VREPSIM_LIB_SRC
  # vrep remote api
  vrep_lib/remoteApi/extApi.c
  vrep_lib/remoteApi/extApiPlatform.c
)

add_library(vrep_sim STATIC ${VREPSIM_LIB_SRC})
add_definitions(-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255)
#target_compile_definitions(vrep_sim PUBLIC NON_MATLAB_PARSING MAX_EXT_API_CONNECTIONS=1)

if (UNIX)
 	target_link_libraries(vrep_sim pthread)
endif (UNIX)
