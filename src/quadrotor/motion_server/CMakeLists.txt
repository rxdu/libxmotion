# Include path
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
set(QUAD_LIB_SRC
    motion_server.cpp
    quad_flattraj_handler.cpp
    quad_polyopt.cpp
)
add_library(quad_motion_server STATIC ${QUAD_LIB_SRC})
target_link_libraries(quad_motion_server map polyopt ${OpenCV_LIBS} ${Octomap_LIBS} ${LCM_LIBRARIES})

# Add executables
#add_executable(wp_manager tests/wp_manager.cpp)
#target_link_libraries(wp_manager mission ${LCM_LIBRARIES})