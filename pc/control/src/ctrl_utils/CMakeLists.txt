# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
set(CTRL_UTILS_LIB_SRC
    # logging
    logging/utils_log.cpp
    # vis
    vis/quadvis_data_transmitter.cpp
)
add_library(ctrl_utils STATIC ${CTRL_UTILS_LIB_SRC})
#add_definitions(-DENABLE_LOG)
target_link_libraries(ctrl_utils ${LCM_LIBRARIES})

# Add executables
#add_executable(test_graph test_graph.cpp)
#target_link_libraries(test_graph qtree vis map ${OpenCV_LIBS})
