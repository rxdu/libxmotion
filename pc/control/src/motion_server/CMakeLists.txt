# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
set(MOTION_SERVER_LIB_SRC
    motion_server.cpp
)
add_library(motion_server STATIC ${MOTION_SERVER_LIB_SRC})
target_link_libraries(motion_server ${LCM_LIBRARIES})

# Add executables
add_executable(quad_motion_client motion_client.cpp)
target_link_libraries(quad_motion_client ${LCM_LIBRARIES})
