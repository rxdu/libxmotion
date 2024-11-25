if (TARGET motor_waveshare)
  add_executable(test_swerve_drive_robot test_swerve_drive_robot.cpp)
  target_link_libraries(test_swerve_drive_robot PRIVATE robot_base motor_waveshare)
endif ()

if (TARGET motor_akelc AND TARGET input_hid)
  add_executable(test_differential_drive_robot test_differential_drive_robot.cpp)
  target_link_libraries(test_differential_drive_robot PRIVATE robot_base motor_akelc input_hid)
endif ()

add_executable(test_swerve_drive_kinematics test_swerve_drive_kinematics.cpp)
target_link_libraries(test_swerve_drive_kinematics PRIVATE robot_base)