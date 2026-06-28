if (TARGET xmotion::motor_waveshare)
  add_executable(test_swerve_drive_robot test_swerve_drive_robot.cpp)
  target_link_libraries(test_swerve_drive_robot PRIVATE robot_base xmotion::motor_waveshare)
endif ()

if (TARGET xmotion::motor_akelc AND TARGET xmotion::input_hid)
  add_executable(test_differential_drive_robot test_differential_drive_robot.cpp)
  target_link_libraries(test_differential_drive_robot PRIVATE robot_base xmotion::motor_akelc xmotion::input_hid)
endif ()

add_executable(test_swerve_drive_kinematics test_swerve_drive_kinematics.cpp)
target_link_libraries(test_swerve_drive_kinematics PRIVATE robot_base)