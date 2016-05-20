# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
set(QUADCON_LIB_SRC
    # state estimator
    estimator/att_estimator.cpp
    estimator/robot_state.cpp
    # controller
    controller/att_euler_con.cpp
    controller/pos_euler_con.cpp
    controller/att_quat_con.cpp
    controller/pos_quat_con.cpp
    # motion management
    motion/trajectory_manager.cpp
)
add_library(quad_con STATIC ${QUADCON_LIB_SRC})
if (ENABLE_G3LOG)
	add_definitions(-DENABLE_LOG)
  target_link_libraries(quad_con ctrl_utils g3logger)
endif (ENABLE_G3LOG)

# Add executables
#add_executable(test_graph test_graph.cpp)
#target_link_libraries(test_graph qtree vis map ${OpenCV_LIBS})
