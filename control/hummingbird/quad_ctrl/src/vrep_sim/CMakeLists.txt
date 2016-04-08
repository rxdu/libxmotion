# Include path
include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_SOURCE_DIR}/src/control)
include_directories(${CMAKE_SOURCE_DIR}/src/vrep_remote_api/include)
include_directories(${CMAKE_SOURCE_DIR}/src/vrep_remote_api/remoteApi)
include_directories(${CMAKE_SOURCE_DIR}/src/g3log_srcl/src)

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
set(VREPSIM_LIB_SRC
  vrep_client/quad_sim_client.cpp
  sim_process/quad_sim_process.cpp
  utils/utils_log.cpp
)
add_library(vrep_sim STATIC ${VREPSIM_LIB_SRC})
add_definitions(-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255)
#target_link_libraries(vis ${OpenCV_LIBS})

# Add executables
add_executable(quad_vrep_sim main.cpp)
target_link_libraries(quad_vrep_sim vrep_sim vrep_remote_api quad_con g3logger)
