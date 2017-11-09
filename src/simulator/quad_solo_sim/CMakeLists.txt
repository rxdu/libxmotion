# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(QUADSIM_LIB_SRC
    quad_solo_sim_client.cpp
    quad_solo_sim_control.cpp
)
add_library(quadsolosim STATIC ${QUADSIM_LIB_SRC})
add_definitions(-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255)
target_link_libraries(quadsolosim vrep_sim quad_ctrl quad_base ${LCM_LIBRARIES})

# Add executables
#add_executable(quadsim_demo quadsim_demo.cpp)
#target_link_libraries(quadsim_demo quadsim quad_ctrl)
