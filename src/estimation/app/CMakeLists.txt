find_package(ament_cmake QUIET)

if (ament_cmake_FOUND)
    message(STATUS "Found ament_cmake, building app_mekf_estimator")

    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)

    add_executable(app_mekf_estimator
            app_mekf_estimator.cpp
            mekf_ros_node.cpp)
    target_link_libraries(app_mekf_estimator PRIVATE
            sensor_imu
            ${rclcpp_LIBRARIES}
            ${sensor_msgs_LIBRARIES})
    target_include_directories(app_mekf_estimator PRIVATE
            ${rclcpp_INCLUDE_DIRS}
            ${sensor_msgs_INCLUDE_DIRS})
else ()
    message(STATUS "ament_cmake not found, not building app_mekf_estimator")
endif ()