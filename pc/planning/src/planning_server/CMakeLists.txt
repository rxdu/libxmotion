# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
set(PLANNING_SERVER_LIB_SRC
    map_server.cpp
)
add_library(planning_server STATIC ${PLANNING_SERVER_LIB_SRC})
target_link_libraries(planning_server ${LCM_LIBRARIES})

# Add executables
#add_executable(quad_motion_client motion_client.cpp)
#target_link_libraries(quad_motion_client ${LCM_LIBRARIES})
