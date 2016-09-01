# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../vrep_sim/vrep_lib/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../vrep_sim/vrep_lib/remoteApi)

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
set(QUADSIM_LIB_SRC
    quad_sim_client.cpp
)
add_library(quadsim STATIC ${QUADSIM_LIB_SRC})
if (ENABLE_G3LOG)
	add_definitions(-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -DENABLE_LOG)
else (ENABLE_G3LOG)
	add_definitions(-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255)
endif (ENABLE_G3LOG)
target_link_libraries(quadsim vrep_sim)

# Add executables
add_executable(quadsim_demo quadsim_demo.cpp)
target_link_libraries(quadsim_demo quadsim)
