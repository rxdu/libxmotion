# Include path
include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_SOURCE_DIR}/src/g3log_srcl/src)

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
)
add_library(quad_con STATIC ${QUADCON_LIB_SRC})
#target_link_libraries(vis ${OpenCV_LIBS})

# Add executables
#add_executable(test_graph test_graph.cpp)
#target_link_libraries(test_graph qtree vis map ${OpenCV_LIBS})
