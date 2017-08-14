# Include path
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
set(QUAD_DRIVER_SRC
    quad_data_broadcaster.cpp
)
add_library(quad_driver STATIC ${QUAD_DRIVER_SRC})
target_link_libraries(quad_driver ${LCM_LIBRARIES})
