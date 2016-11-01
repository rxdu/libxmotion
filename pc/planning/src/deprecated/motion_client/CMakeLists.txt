# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
set(MOTION_CLIENT_LIB_SRC
    poly_motion_client.cpp
)
add_library(motion_client STATIC ${MOTION_CLIENT_LIB_SRC})
target_link_libraries(motion_client map local3d planner utils polyopt quadflat ${OpenCV_LIBS} ${Octomap_LIBS} ${LCM_LIBRARIES})

# Add executables
add_executable(poly_client tests/test_polyclient.cpp)
target_link_libraries(poly_client map local3d utils motion_client ${Octomap_LIBS} ${LCM_LIBRARIES})