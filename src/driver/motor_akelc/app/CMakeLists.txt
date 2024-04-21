find_package(ament_cmake QUIET)

if (ament_cmake_FOUND)
  message(STATUS "Found ament_cmake, building app_akelc_motor_tool")

  find_package(rclcpp REQUIRED)
  find_package(rclcpp_action REQUIRED)
  find_package(sensor_msgs REQUIRED)

  add_executable(app_akelc_motor_tool
      app_akelc_motor_tool.cpp)
  target_link_libraries(app_akelc_motor_tool PUBLIC
      motor_akelc
      ${rclcpp_LIBRARIES}
      ${rclcpp_action_LIBRARIES}
      ${sensor_msgs_LIBRARIES})
  target_include_directories(app_akelc_motor_tool PUBLIC
      ${rclcpp_INCLUDE_DIRS}
      ${rclcpp_action_INCLUDE_DIRS}
      ${sensor_msgs_INCLUDE_DIRS})
else ()
  message(STATUS "ament_cmake not found, not building app_akelc_motor_tool")
endif ()